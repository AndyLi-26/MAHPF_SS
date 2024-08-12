import os
import shutil

def organize_map_files():
    # Get the current working directory
    directory = os.getcwd()

    # List all files in the directory
    files = os.listdir(directory)

    # Iterate through all files
    for file in files:
        # Check if the file is a .map file
        if file.endswith('.map'):
            # Extract the file name without the extension
            folder_name = os.path.splitext(file)[0]

            # Create a new directory with the file's origin name
            if not os.path.exists(folder_name):
                os.mkdir(folder_name)

            # Move the .map file into the new directory
            shutil.move(file, os.path.join(folder_name, file))
            print(f"Moved {file} to {folder_name}/")
            os.mkdir(os.path.join(folder_name, "scen"))
            os.mkdir(os.path.join(folder_name, "scen", "robot"))
            os.mkdir(os.path.join(folder_name, "scen", "human"))


# Run the function to organize .map files in the current directory
organize_map_files()

