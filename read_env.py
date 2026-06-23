import os
try:
    from SCons.Script import DefaultEnvironment
except ImportError:
    # Fallback to allow IDE linters to parse/run this script without errors
    class DummyEnv:
        def get(self, key, default=None):
            return default
        def Append(self, **kwargs):
            pass
    def DefaultEnvironment():
        return DummyEnv()

env = DefaultEnvironment()

# Read .env file in the project root
ssid = ""
password = ""
env_path = os.path.join(env.get("PROJECT_DIR", "."), ".env")

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
                if key == "WIFI_SSID":
                    ssid = val
                elif key == "WIFI_PASS":
                    password = val

if ssid and password:
    print(f"Injecting credentials from .env: WIFI_SSID={ssid}")
    env.Append(CPPDEFINES=[
        ("WIFI_SSID", f'\\"{ssid}\\"'),
        ("WIFI_PASS", f'\\"{password}\\"')
    ])
else:
    print("Warning: WIFI_SSID or WIFI_PASS not found in .env!")
