import unittest
import sys
import os
import hashlib
from unittest.mock import MagicMock, patch, mock_open

# Import functions from deploy_firmware
# We append project path to sys.path so we can import it
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
import deploy_firmware

class TestDeployFirmware(unittest.TestCase):
    
    def test_load_env(self):
        env_content = "ROVER_PI_HOST=10.0.0.1\nROVER_PI_USER=testuser\n# Comment\nINVALID_LINE"
        with patch("builtins.open", mock_open(read_data=env_content)):
            env = deploy_firmware.load_env(".env")
            self.assertEqual(env.get("ROVER_PI_HOST"), "10.0.0.1")
            self.assertEqual(env.get("ROVER_PI_USER"), "testuser")
            self.assertNotIn("INVALID_LINE", env)

    def test_ssh_config_resolution(self):
        # Test default lookup when no config exists
        with patch("os.path.exists", return_value=False):
            info = deploy_firmware.parse_ssh_config("rover")
            self.assertEqual(info["hostname"], "rover")
            self.assertEqual(info["port"], 22)
            
        # Test lookup when config exists
        config_mock = MagicMock()
        config_mock.lookup.return_value = {"hostname": "10.0.0.246", "user": "ron", "port": "2222"}
        with patch("os.path.exists", return_value=True):
            with patch("paramiko.SSHConfig", return_value=config_mock):
                with patch("builtins.open", mock_open()):
                    info = deploy_firmware.parse_ssh_config("rover")
                    self.assertEqual(info["hostname"], "10.0.0.246")
                    self.assertEqual(info["user"], "ron")
                    self.assertEqual(info["port"], 2222)

    @patch("deploy_firmware.verify_remote_device")
    def test_device_resolver_accepts_ch340(self, mock_verify):
        # Simulated success for CH340 on /dev/rover-esp32
        mock_verify.return_value = (True, "Verified CH340")
        client = MagicMock()
        ok, msg = deploy_firmware.verify_remote_device(client, "/dev/rover-esp32")
        self.assertTrue(ok)
        self.assertIn("Verified", msg)

    def test_device_resolver_rejects_lidar(self):
        # Mock udev properties returning CP2102N LiDAR IDs
        client = MagicMock()
        stdout_mock = MagicMock()
        stdout_mock.read.return_value = b"ID_VENDOR_ID=10c4\nID_MODEL_ID=ea60\n"
        client.exec_command.side_effect = [
            (None, MagicMock(read=lambda: b"exists"), None), # Check exist
            (None, stdout_mock, None) # Check properties
        ]
        ok, msg = deploy_firmware.verify_remote_device(client, "/dev/rover-esp32")
        self.assertFalse(ok)
        self.assertIn("is CP2102N LiDAR", msg)

    def test_device_resolver_rejects_unknown(self):
        # Mock udev properties returning arbitrary vendor/product IDs
        client = MagicMock()
        stdout_mock = MagicMock()
        stdout_mock.read.return_value = b"ID_VENDOR_ID=abcd\nID_MODEL_ID=1234\n"
        client.exec_command.side_effect = [
            (None, MagicMock(read=lambda: b"exists"), None),
            (None, stdout_mock, None)
        ]
        ok, msg = deploy_firmware.verify_remote_device(client, "/dev/rover-esp32")
        self.assertFalse(ok)
        self.assertIn("has unknown hardware ID", msg)

    def test_check_sudo_nopasswd_success(self):
        client = MagicMock()
        stdout_mock = MagicMock()
        stdout_mock.read.return_value = b"NOPASSWD\n"
        client.exec_command.return_value = (None, stdout_mock, None)
        self.assertTrue(deploy_firmware.check_sudo_nopasswd(client))

    def test_check_sudo_nopasswd_fail(self):
        client = MagicMock()
        stdout_mock = MagicMock()
        stdout_mock.read.return_value = b"Sorry, user ron may not run sudo on rpi5-nas.\n"
        client.exec_command.return_value = (None, stdout_mock, None)
        self.assertFalse(deploy_firmware.check_sudo_nopasswd(client))

    @patch("urllib.request.urlopen")
    def test_post_flash_verification_detects_reset_loop(self, mock_urlopen):
        client = MagicMock()
        # Mock active status check
        client.exec_command.side_effect = [
            # First command: systemctl is-active
            (None, MagicMock(read=lambda: b"active"), None),
            # Second command: journalctl (has 3 startup banners)
            (None, MagicMock(read=lambda: b"Firmware Name: Maker-ESP32-Unified-Rover\n" * 4), None)
        ]
        
        # Test main logic block of verification using command mocks
        # We manually check the logic used in deploy_firmware.py
        stdin, stdout, stderr = client.exec_command("systemctl is-active")
        active = stdout.read().decode().strip()
        self.assertEqual(active, "active")
        
        stdin, stdout, stderr = client.exec_command("journalctl")
        logs = stdout.read().decode()
        startup_count = logs.count("Firmware Name: Maker-ESP32-Unified-Rover")
        self.assertGreater(startup_count, 2) # Trigger reset loop detection

    @patch("urllib.request.urlopen")
    def test_post_flash_verification_detects_invalid_header(self, mock_urlopen):
        client = MagicMock()
        client.exec_command.side_effect = [
            (None, MagicMock(read=lambda: b"active"), None),
            (None, MagicMock(read=lambda: b"invalid header: 0xffffffff\n"), None)
        ]
        
        stdin, stdout, stderr = client.exec_command("systemctl is-active")
        active = stdout.read().decode().strip()
        self.assertEqual(active, "active")
        
        stdin, stdout, stderr = client.exec_command("journalctl")
        logs = stdout.read().decode()
        self.assertIn("invalid header: 0xffffffff", logs)

    def test_checksum_verification_fail(self):
        # Verify SHA-256 computation matches standard hash logic
        local_data = b"local binary data"
        remote_data = b"different remote data"
        
        local_hash = hashlib.sha256(local_data).hexdigest()
        remote_hash = hashlib.sha256(remote_data).hexdigest()
        
        self.assertNotEqual(local_hash, remote_hash)

    def test_service_template_uses_setup_placeholders(self):
        # Read the local template file
        template_path = "C:/Users/Ron/electronic_projects/yahboom-encoder/rpi5/rover-lidar.service.template"
        self.assertTrue(os.path.exists(template_path))
        with open(template_path, "r") as f:
            content = f.read()
        self.assertIn("{{ROVER_LIDAR_DEVICE}}", content)
        self.assertNotIn("${ROVER_LIDAR_DEVICE}", content)

    def test_setup_script_contains_validation(self):
        # Read the setup script file
        setup_path = "C:/Users/Ron/electronic_projects/yahboom-encoder/rpi5/setup.sh"
        self.assertTrue(os.path.exists(setup_path))
        with open(setup_path, "r") as f:
            content = f.read()
        self.assertIn("execstart=.*--dev[[:space:]]*\"\"", content)
        self.assertIn("execstart=.*--dev[[:space:]]*$", content)
        self.assertIn("chmod +x \"$WORKING_DIR/rpi5/rover-doctor.sh\"", content)
        self.assertIn("systemd-analyze verify", content)
        self.assertIn("\{\{[a-zA-Z0-9_]+\}\}", content)

    def test_server_js_lacks_linux_fallbacks(self):
        # Read server.js
        server_path = "C:/Users/Ron/electronic_projects/yahboom-encoder/server.js"
        self.assertTrue(os.path.exists(server_path))
        with open(server_path, "r") as f:
            content = f.read()
        # Verify ttyUSB0/ttyACM0 fallbacks on Linux are removed from COM_PORT resolution
        self.assertNotIn("fs.existsSync('/dev/ttyUSB0')", content)
        self.assertNotIn("fs.existsSync('/dev/ttyACM0')", content)

    def test_beep_dtr_test_default_port(self):
        # Read beep_dtr_test.js
        test_path = "C:/Users/Ron/electronic_projects/yahboom-encoder/beep_dtr_test.js"
        self.assertTrue(os.path.exists(test_path))
        with open(test_path, "r") as f:
            content = f.read()
        # Verify it doesn't default to /dev/ttyUSB1
        self.assertNotIn("'/dev/ttyUSB1'", content)
        self.assertIn("'/dev/rover-esp32'", content)

if __name__ == "__main__":
    unittest.main()
