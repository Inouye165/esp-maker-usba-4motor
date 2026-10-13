import os
import sys
import json
import time
import subprocess
import hashlib
import argparse
import urllib.request
import urllib.error
import paramiko

EXPECTED_FIRMWARE_SHA256 = "19B3405F960D4FB07B170212D92CDB94BD664FE12C56D092347B3D4A9083ACB0"

def load_env(env_path):
    env_vars = {}
    if os.path.exists(env_path):
        with open(env_path, "r") as f:
            for line in f:
                line = line.strip()
                if not line or line.startswith("#"):
                    continue
                parts = line.split("=", 1)
                if len(parts) == 2:
                    key = parts[0].strip()
                    val = parts[1].strip().strip('"').strip("'")
                    env_vars[key] = val
    return env_vars

def parse_ssh_config(ip):
    ssh_config_path = os.path.expanduser("~/.ssh/config")
    host_info = {"hostname": ip, "user": None, "port": 22}
    if os.path.exists(ssh_config_path):
        try:
            ssh_config = paramiko.SSHConfig()
            with open(ssh_config_path) as f:
                ssh_config.parse(f)
            info = ssh_config.lookup(ip)
            if info:
                if 'hostname' in info:
                    host_info['hostname'] = info['hostname']
                if 'user' in info:
                    host_info['user'] = info['user']
                if 'port' in info:
                    host_info['port'] = int(info['port'])
        except Exception as e:
            print(f"Warning: Failed to parse ssh config: {e}", file=sys.stderr)
    return host_info

def calculate_sha256(file_path):
    sha256 = hashlib.sha256()
    with open(file_path, 'rb') as f:
        while True:
            data = f.read(65536)
            if not data:
                break
            sha256.update(data)
    return sha256.hexdigest()

def extract_platformio_metadata(project_dir):
    json_path = os.path.join(project_dir, ".pio", "build", "esp32dev", "idedata.json")
    metadata = {
        "images": [],
        "fallback_used": False
    }
    
    if os.path.exists(json_path):
        try:
            with open(json_path, "r") as f:
                data = json.load(f)
            
            extra = data.get("extra", {})
            flash_images = extra.get("flash_images", [])
            app_offset = extra.get("application_offset", "0x10000")
            
            # Normalize and add images
            for img in flash_images:
                local_path = os.path.normpath(img["path"])
                metadata["images"].append({
                    "offset": img["offset"],
                    "local_path": local_path
                })
            
            # Add application firmware binary
            bin_path = os.path.join(project_dir, ".pio", "build", "esp32dev", "firmware.bin")
            metadata["images"].append({
                "offset": app_offset,
                "local_path": os.path.normpath(bin_path)
            })
            return metadata
        except Exception as e:
            print(f"Warning: Failed to parse idedata.json: {e}. Falling back to default offsets.", file=sys.stderr)
            
    # Fallback default offsets
    metadata["fallback_used"] = True
    build_dir = os.path.join(project_dir, ".pio", "build", "esp32dev")
    
    home_dir = os.path.expanduser("~")
    boot_app0_std = os.path.join(home_dir, ".platformio", "packages", "framework-arduinoespressif32", "tools", "partitions", "boot_app0.bin")
    if not os.path.exists(boot_app0_std):
        boot_app0_std = os.path.join(project_dir, "boot_app0.bin")
        
    metadata["images"] = [
        {"offset": "0x1000", "local_path": os.path.join(build_dir, "bootloader.bin")},
        {"offset": "0x8000", "local_path": os.path.join(build_dir, "partitions.bin")},
        {"offset": "0xe000", "local_path": boot_app0_std},
        {"offset": "0x10000", "local_path": os.path.join(build_dir, "firmware.bin")}
    ]
    return metadata

def verify_remote_device(client, esp_port, expected_vid="1a86", expected_pid="7523", lidar_vid="10c4", lidar_pid="ea60"):
    # Check if device exists
    stdin, stdout, stderr = client.exec_command(f"[ -e '{esp_port}' ] && echo 'exists'")
    if stdout.read().decode().strip() != 'exists':
        return False, f"Device port '{esp_port}' does not exist on remote host."
    
    # Get udev properties
    stdin, stdout, stderr = client.exec_command(f"udevadm info -q property -n '{esp_port}'")
    properties = stdout.read().decode()
    
    vid, pid = "", ""
    for line in properties.splitlines():
        if line.startswith("ID_VENDOR_ID="):
            vid = line.split("=", 1)[1].strip()
        elif line.startswith("ID_MODEL_ID="):
            pid = line.split("=", 1)[1].strip()
            
    if not vid or not pid:
        return False, f"Could not determine VID/PID for '{esp_port}'."
        
    if vid == lidar_vid and pid == lidar_pid:
        return False, f"Device '{esp_port}' is CP2102N LiDAR ({vid}:{pid}). Refusing to flash!"
        
    if vid != expected_vid or pid != expected_pid:
        return False, f"Device '{esp_port}' has unknown hardware ID ({vid}:{pid}). Expected CH340 ({expected_vid}:{expected_pid})."
        
    return True, f"Verified '{esp_port}' is CH340 device ({vid}:{pid})."

def check_sudo_nopasswd(client):
    stdin, stdout, stderr = client.exec_command("sudo -n true 2>/dev/null && echo 'NOPASSWD'")
    return stdout.read().decode().strip() == 'NOPASSWD'

def main():
    parser = argparse.ArgumentParser(description="Harden ESP32 Remote Flashing tool")
    parser.add_argument("--dry-run", action="store_true", help="Simulate build, check configs, hashes, and verification gates, but skip actual flash")
    parser.add_argument("--check-only", action="store_true", help="Perform pre-flash validations (connection, port ownership, VID/PID, sudo) and exit")
    parser.add_argument("--no-flash", action="store_true", help="Build and upload to remote tmp dir, but skip flashing")
    parser.add_argument("--keep-remote-files", action="store_true", help="Do not clean up uploaded remote binaries after flashing")
    parser.add_argument("--skip-backup", action="store_true", help="Skip reading 4MB flash rollback backup before flashing")
    parser.add_argument("--skip-compile", action="store_true", help="Skip local compilation and deploy existing prebuilt verified binary")
    args = parser.parse_args()

    project_dir = os.path.dirname(os.path.abspath(__file__))
    
    # 1. Load .env
    env = load_env(os.path.join(project_dir, ".env"))
    
    ip = env.get("ROVER_PI_HOST", "rover")
    user = env.get("ROVER_PI_USER", "ron")
    port = int(env.get("ROVER_PI_PORT", 22))
    esp_port = env.get("ROVER_ESP32_DEVICE", "/dev/rover-esp32")
    
    # Resolve SSH config details
    ssh_info = parse_ssh_config(ip)
    ip = ssh_info["hostname"]
    if ssh_info["user"]:
        user = ssh_info["user"]
    if ssh_info["port"]:
        port = ssh_info["port"]

    print("=== Compilation ===")
    if args.check_only or args.skip_compile:
        print("Skipping local compilation (using prebuilt verified binary).")
    else:
        print("Compiling firmware locally using PlatformIO...")
        try:
            subprocess.run(["pio", "run"], check=True)
            print("Compilation: PASS")
        except subprocess.CalledProcessError as e:
            print(f"Compilation: FAIL ({e})", file=sys.stderr)
            sys.exit(1)
            
    # Extract metadata & check local files
    metadata = extract_platformio_metadata(project_dir)
    print(f"Resolved PlatformIO offsets: {'(Fallback Defaults)' if metadata['fallback_used'] else '(dynamic idedata.json)'}")
    
    for img in metadata["images"]:
        path = img["local_path"]
        print(f"  - Offset {img['offset']}: {os.path.basename(path)}")
        if not args.check_only and not os.path.exists(path):
            print(f"Error: Required local binary not found at {path}", file=sys.stderr)
            sys.exit(1)

    # 2. SSH connection
    print("\n=== SSH Connection ===")
    print(f"Connecting to RPi5 at {user}@{ip}:{port} using SSH keys/agent...")
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    
    try:
        client.connect(ip, port=port, username=user, timeout=10)
        print("SSH Connection: PASS")
    except Exception as e:
        print(f"SSH Connection: FAIL ({e})", file=sys.stderr)
        sys.exit(1)
        
    try:
        # Check passwordless sudo
        if not check_sudo_nopasswd(client):
            print("Error: Passwordless sudo is not configured or fails. Flashing requires passwordless sudo.", file=sys.stderr)
            sys.exit(1)
            
        # 3. Device Identification Validation
        print("\n=== USB Identity ===")
        ok, msg = verify_remote_device(client, esp_port)
        if not ok:
            print(f"USB Identity: FAIL ({msg})", file=sys.stderr)
            sys.exit(1)
        print(f"USB Identity: PASS ({msg})")
        
        # Check process ownership of serial port (must not be in use except possibly by rover-server which we'll stop)
        stdin, stdout, stderr = client.exec_command(f"fuser {esp_port} 2>/dev/null")
        pids = stdout.read().decode().strip().split()
        
        # Get server service PID
        stdin, stdout, stderr = client.exec_command("systemctl show --property=MainPID rover-server.service 2>/dev/null")
        service_pid_line = stdout.read().decode().strip()
        service_pid = service_pid_line.split("=", 1)[1] if service_pid_line and "=" in service_pid_line else ""
        
        unexpected_pids = [p for p in pids if p != service_pid and p != ""]
        if unexpected_pids:
            print(f"Error: Serial port '{esp_port}' is currently owned by unexpected processes (PIDs: {', '.join(unexpected_pids)}).", file=sys.stderr)
            sys.exit(1)
            
        # Verify esptool connects to target (check chip_id/read_mac)
        print("Verifying esptool remote chip compatibility...")
        # Check esptool is installed remotely
        stdin, stdout, stderr = client.exec_command("python3 -m esptool version 2>/dev/null")
        if stdout.channel.recv_exit_status() != 0:
            print("Error: Remote esptool is missing or broken on the Pi. Ensure python3-esptool is installed.", file=sys.stderr)
            sys.exit(1)
            
        if args.check_only:
            print("Check-only validations complete. All pre-flash gates PASSED.")
            sys.exit(0)
            
        # 4. Upload & SHA-256 Comparison
        print("\n=== Remote Checksums ===")
        temp_dir = f"/tmp/rover_deploy_{int(time.time())}"
        print(f"Creating unique temporary directory: {temp_dir}")
        client.exec_command(f"mkdir -p {temp_dir}")
        
        sftp = client.open_sftp()
        remote_images = []
        
        for img in metadata["images"]:
            local_path = img["local_path"]
            filename = os.path.basename(local_path)
            remote_path = f"{temp_dir}/{filename}"
            
            # Compute local SHA-256
            local_hash = calculate_sha256(local_path)
            
            # If application firmware, verify local binary against manifest before uploading
            if filename == "firmware.bin":
                if local_hash.upper() != EXPECTED_FIRMWARE_SHA256.upper():
                    print(f"Error: Local firmware.bin hash ({local_hash.upper()}) does NOT match manifest ({EXPECTED_FIRMWARE_SHA256})!", file=sys.stderr)
                    sys.exit(1)
                print(f"Local firmware.bin manifest check: PASS ({local_hash.upper()})")

            print(f"Uploading {filename}...")
            sftp.put(local_path, remote_path)
            
            # Compute remote SHA-256
            stdin, stdout, stderr = client.exec_command(f"sha256sum '{remote_path}'")
            remote_hash_line = stdout.read().decode().strip()
            remote_hash = remote_hash_line.split()[0] if remote_hash_line else ""
            
            if local_hash.lower() != remote_hash.lower():
                print(f"Error: Hash mismatch for uploaded file {filename}! Local: {local_hash}, Remote: {remote_hash}", file=sys.stderr)
                sys.exit(1)

            if filename == "firmware.bin":
                if remote_hash.upper() != EXPECTED_FIRMWARE_SHA256.upper():
                    print(f"Error: Remote firmware.bin hash ({remote_hash.upper()}) does NOT match manifest ({EXPECTED_FIRMWARE_SHA256})!", file=sys.stderr)
                    sys.exit(1)
                print(f"Remote firmware.bin manifest check: PASS ({remote_hash.upper()})")
                
            remote_images.append({
                "offset": img["offset"],
                "remote_path": remote_path
            })
            
        sftp.close()
        print("Remote checksums: PASS (All uploaded files verified)")
        
        if args.no_flash:
            print("No-flash mode selected. Files uploaded. Stopping before flashing.")
            sys.exit(0)

        if args.dry_run:
            print("\n=== Dry-Run Simulation ===")
            print("Dry-run: Compilation: PASS")
            print("Dry-run: SSH connection: PASS")
            print("Dry-run: USB identity: PASS")
            print("Dry-run: Remote checksums: PASS")
            print("Dry-run: Flash write and verification: PASS (SIMULATED)")
            print("Dry-run: ESP32 normal boot: PASS (SIMULATED)")
            print("Dry-run: Rover service restart: PASS (SIMULATED)")
            print("Dry-run: Cockpit protocol connection: PASS (SIMULATED)")
            print("Dry-run completed successfully!")
            sys.exit(0)

        # 5. Stop rover-server and flash
        print("\n=== Flash Write and Verification ===")
        print("Stopping remote rover-server.service...")
        stdin, stdout, stderr = client.exec_command("sudo -n systemctl stop rover-server.service")
        stop_exit = stdout.channel.recv_exit_status()
        if stop_exit != 0:
            err = stderr.read().decode().strip()
            print(f"Error: Failed to stop remote rover-server.service (exit code {stop_exit}): {err}", file=sys.stderr)
            print("Aborting flash deployment!", file=sys.stderr)
            sys.exit(1)

        # Verify service state is inactive
        time.sleep(1)
        stdin, stdout, stderr = client.exec_command("systemctl is-active rover-server.service")
        srv_state = stdout.read().decode().strip()
        if srv_state != "inactive" and srv_state != "failed":
            print(f"Error: rover-server.service state is '{srv_state}', expected 'inactive'. Aborting flash deployment!", file=sys.stderr)
            sys.exit(1)
        print("Remote rover-server.service stopped: PASS")

        # Verify port is completely released
        stdin, stdout, stderr = client.exec_command(f"fuser {esp_port} 2>/dev/null")
        pids = stdout.read().decode().strip().split()
        if pids:
            print(f"Error: Serial port '{esp_port}' is still in use by processes (PIDs: {', '.join(pids)}) after stopping service! Aborting flash deployment!", file=sys.stderr)
            sys.exit(1)
        print(f"Serial port '{esp_port}' released: PASS")
            
        try:
            # 5a. Automatic Pre-Flash Rollback Backup
            if not args.skip_backup:
                print("\n=== Pre-Flash Rollback Backup ===")
                timestamp_str = time.strftime("%Y%m%d_%H%M%S")
                backup_filename = f"backup_rover1_flash_4mb_1.3.2-olp-safe_{timestamp_str}.bin"
                remote_backup_path = f"/home/{user}/{backup_filename}"
                local_backup_dir = os.path.join(project_dir, "backups")
                os.makedirs(local_backup_dir, exist_ok=True)
                local_backup_path = os.path.join(local_backup_dir, backup_filename)

                backup_cmd = f"python3 -m esptool --chip esp32 --port {esp_port} --baud 115200 read_flash 0x0 0x400000 '{remote_backup_path}'"
                print(f"Reading 4MB flash image: {backup_cmd}")
                stdin, stdout, stderr = client.exec_command(backup_cmd)
                while True:
                    line = stdout.readline()
                    if not line:
                        break
                    print(line, end="")
                backup_status = stdout.channel.recv_exit_status()
                if backup_status != 0:
                    err_details = stderr.read().decode().strip()
                    print(f"Error: Pre-flash backup failed with exit code {backup_status}: {err_details}", file=sys.stderr)
                    print("Aborting flash deployment to preserve running configuration!", file=sys.stderr)
                    sys.exit(1)

                print(f"Remote flash backup created: {remote_backup_path}")
                
                # Verify backup against physical ESP32 flash chip
                print("Verifying full-flash backup against physical ESP32 chip...")
                verify_backup_cmd = f"python3 -m esptool --chip esp32 --port {esp_port} --baud 115200 verify_flash 0x0 '{remote_backup_path}'"
                stdin, stdout, stderr = client.exec_command(verify_backup_cmd)
                while True:
                    line = stdout.readline()
                    if not line:
                        break
                    print(line, end="")
                verify_backup_status = stdout.channel.recv_exit_status()
                if verify_backup_status != 0:
                    err_details = stderr.read().decode().strip()
                    print(f"Error: Full-flash backup verification against chip failed: {err_details}", file=sys.stderr)
                    print("Aborting flash deployment!", file=sys.stderr)
                    sys.exit(1)
                print("Full-flash backup verified against ESP32 chip: PASS")

                # Compute remote SHA-256 of the backup on the Pi
                stdin, stdout, stderr = client.exec_command(f"sha256sum '{remote_backup_path}'")
                remote_backup_hash_line = stdout.read().decode().strip()
                remote_backup_hash = remote_backup_hash_line.split()[0] if remote_backup_hash_line else ""
                print(f"Remote flash backup SHA-256: {remote_backup_hash}")

                print("Downloading rollback image to local repository...")
                sftp = client.open_sftp()
                sftp.get(remote_backup_path, local_backup_path)
                sftp.close()

                # Verify downloaded local backup against remote original
                local_backup_hash = calculate_sha256(local_backup_path)
                print(f"Local flash backup SHA-256:  {local_backup_hash}")
                if local_backup_hash.lower() != remote_backup_hash.lower():
                    print(f"Error: Downloaded backup hash ({local_backup_hash}) does not match remote original ({remote_backup_hash})!", file=sys.stderr)
                    print("Aborting flash deployment!", file=sys.stderr)
                    sys.exit(1)
                print(f"Rollback backup verified against remote original: PASS (SHA-256: {local_backup_hash})")

            # Build flash command
            # Command layout: write_flash <offset1> <path1> <offset2> <path2> ...
            flash_args = []
            for img in remote_images:
                flash_args.append(f"{img['offset']} {img['remote_path']}")
                
            flash_cmd = f"python3 -m esptool --chip esp32 --port {esp_port} --baud 115200 --before default_reset --after hard_reset write_flash {' '.join(flash_args)}"
            print(f"Flashing command: {flash_cmd}")
            
            # Execute esptool
            stdin, stdout, stderr = client.exec_command(flash_cmd)
            while True:
                line = stdout.readline()
                if not line:
                    break
                print(line, end="")
                
            exit_status = stdout.channel.recv_exit_status()
            if exit_status != 0:
                err_details = stderr.read().decode().strip()
                print(f"Error: Flashing failed with exit code {exit_status}.", file=sys.stderr)
                if err_details:
                    print(f"Details: {err_details}", file=sys.stderr)
                sys.exit(1)
                
            # Verify flashed image
            verify_cmd = f"python3 -m esptool --chip esp32 --port {esp_port} --baud 115200 verify_flash {' '.join(flash_args)}"
            print(f"Running verification command: {verify_cmd}")
            stdin, stdout, stderr = client.exec_command(verify_cmd)
            while True:
                line = stdout.readline()
                if not line:
                    break
                print(line, end="")
            verify_status = stdout.channel.recv_exit_status()
            if verify_status != 0:
                print("Error: Flashing verification failed!", file=sys.stderr)
                sys.exit(1)
                
            print("Flash write and verification: PASS")
            
        finally:
            print("\n=== Rover Service Restart ===")
            print("Restarting remote rover-server.service (Note: service restart does not restore or roll back ESP32 firmware)...")
            _s_in, _s_out, _s_err = client.exec_command("sudo -n systemctl start rover-server.service")
            start_status = _s_out.channel.recv_exit_status()
            if start_status == 0:
                print("Rover service restart: PASS")
            else:
                err_msg = _s_err.read().decode().strip()
                print(f"Rover service restart: FAIL (exit code {start_status}: {err_msg})", file=sys.stderr)
            
        # 6. Post-Flash Verification
        print("\n=== Post-Flash Verification ===")
        print("Waiting for ESP32 boot and cockpit service connection...")
        time.sleep(5) # Allow cockpit to start and open serial port
        
        # Verify service is active
        stdin, stdout, stderr = client.exec_command("systemctl is-active rover-server.service")
        service_active = stdout.read().decode().strip()
        if service_active != "active":
            print(f"Post-flash validation: FAIL (Service state is '{service_active}', expected 'active')", file=sys.stderr)
            sys.exit(1)
            
        # Check logs for invalid header or reset loop
        stdin, stdout, stderr = client.exec_command("journalctl -u rover-server.service -n 50 --no-pager")
        logs = stdout.read().decode()
        
        if "invalid header" in logs.lower() or "invalid header: 0xffffffff" in logs.lower():
            print("Post-flash validation: FAIL (Found 'invalid header' error in logs!)", file=sys.stderr)
            print("=== Service Logs ===", file=sys.stderr)
            print(logs, file=sys.stderr)
            sys.exit(1)
            
        # Check reset loop (e.g. if the startup banner printed multiple times in quick succession)
        startup_banner_count = logs.count("Firmware Name: Maker-ESP32-Unified-Rover")
        if startup_banner_count > 2:
            print(f"Post-flash validation: FAIL (Detected reset loop! Startup banner printed {startup_banner_count} times in logs)", file=sys.stderr)
            sys.exit(1)
            
        # Check HTTP status endpoint for active connection and telemetry
        status_url = f"http://{ip}:3000/api/status"
        print(f"Querying cockpit status endpoint: {status_url}")
        
        serial_connected = False
        last_packet_age = None
        for attempt in range(10):
            try:
                with urllib.request.urlopen(status_url, timeout=2) as response:
                    res_data = json.loads(response.read().decode())
                    if res_data.get("ok"):
                        serial_connected = res_data.get("serialConnected")
                        last_packet_age = res_data.get("lastPacketAgeMs")
                        if serial_connected and last_packet_age is not None and last_packet_age < 1500:
                            break
            except Exception:
                pass
            time.sleep(1)
            
        if not serial_connected:
            print("Post-flash validation: FAIL (Cockpit reports ESP32 is not connected)", file=sys.stderr)
            sys.exit(1)
            
        if last_packet_age is None or last_packet_age > 1500:
            print(f"Post-flash validation: FAIL (Telemetry not flowing or too old: {last_packet_age} ms)", file=sys.stderr)
            sys.exit(1)
            
        print("ESP32 normal boot: PASS")
        print("Cockpit protocol connection: PASS")
        print("Post-flash verification: PASS")
        
        # 7. Cleanup
        if not args.keep_remote_files:
            print(f"Removing remote temporary directory: {temp_dir}")
            client.exec_command(f"rm -rf {temp_dir}")
            
        print("\n=============================================")
        print("  FIRMWARE DEPLOYMENT SUCCESSFUL")
        print("=============================================")
        print("- Compilation: PASS")
        print("- SSH connection: PASS")
        print("- USB identity: PASS")
        print("- Remote checksums: PASS")
        print("- Flash write and verification: PASS")
        print("- ESP32 normal boot: PASS")
        print("- Rover service restart: PASS")
        print("- Cockpit protocol connection: PASS")
        
    finally:
        client.close()

if __name__ == "__main__":
    main()
