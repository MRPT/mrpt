#!/usr/bin/env python3

from mrpt.config import CConfigFile, CConfigFileMemory, CLoadableOptions

# Example with CConfigFile: Reading and writing to a file.
try:
    # Create a CConfigFile object associated with a file.
    config_file = CConfigFile("my_config.ini")

    # Write some values to the config file.
    # The changes are stored in memory until the object is destroyed or writeNow() is called.
    config_file.write("General", "AppName", "MyAwesomeApp")
    config_file.write("General", "Version", 1.0)
    config_file.write("Camera", "FocalLength", 1200.5)
    config_file.write("Lidar", "Port", "/dev/ttyUSB0")

    print("Content written to in-memory config file. The file 'my_config.ini' will be updated upon script exit.")

    # Read values from the config file.
    app_name = config_file.read_string("General", "AppName", "DefaultApp")
    version = config_file.read_double("General", "Version", 0.0)
    focal_length = config_file.read_double("Camera", "FocalLength", 0.0)
    lidar_port = config_file.read_string("Lidar", "Port", "/dev/ttyS0")

    print("\n--- Reading from CConfigFile ---")
    print(f"App Name: {app_name}")
    print(f"Version: {version}")
    print(f"Focal Length: {focal_length}")
    print(f"Lidar Port: {lidar_port}")

    # You can also get a list of sections and keys
    sections = config_file.getAllSections()
    print(f"\nAll sections: {sections}")

    keys_in_general = config_file.getAllKeys("General")
    print(f"Keys in 'General': {keys_in_general}")

except Exception as e:
    print(f"An error occurred with CConfigFile: {e}")

# ---
# Example with CConfigFileMemory: Using a config in memory without a file.
print("\n" + "="*40)
print("Using CConfigFileMemory")
print("="*40)

# Create a CConfigFileMemory object with initial string content.
ini_content = """
[Database]
Host=localhost
Port=5432
User=admin
"""
mem_config = CConfigFileMemory(ini_content)

# Read values from the in-memory config.
db_host = mem_config.read_string("Database", "Host", "127.0.0.1")
db_port = mem_config.read_int("Database", "Port", 3306)
db_user = mem_config.read_string("Database", "User", "guest")

print(f"DB Host: {db_host}")
print(f"DB Port: {db_port}")
print(f"DB User: {db_user}")

# Modify the content.
mem_config.write("Database", "Port", 5433)
new_port = mem_config.read_int("Database", "Port", 3306)
print(f"New DB Port after modification: {new_port}")

# Set new content from a list of strings
new_content_list = [
    "[Network]",
    "Timeout=1000",
    "Protocol=TCP"
]
mem_config.setContent(new_content_list)
timeout = mem_config.read_int("Network", "Timeout", 500)
print(f"Timeout from new content: {timeout}")

# ---
# Example with CLoadableOptions: A custom class with load/save functionality.
print("\n" + "="*40)
print("Using CLoadableOptions")
print("="*40)


# Create an instance and load options from a file.
my_config_file = CConfigFileMemory(
    "[MySection]\nparam1=42\nparam2=3.14\nparam3=hello_world")

print(my_config_file.getContent())
