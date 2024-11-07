import os,shutil
subfolders = ['scen-even', 'scen-random']
# Get the current directory
current_dir = os.getcwd()
# Loop through all items in the current directory
for folder_name in os.listdir(current_dir):
    if folder_name[0]==".": continue
    if ".py" in folder_name: continue
    folder_path = os.path.join(current_dir, folder_name)

    print(folder_path)
    if os.path.isdir(folder_path):
        # Rename the file to map.map
        old_scen_folder_path = os.path.join(folder_path,"robot")
        print(old_scen_folder_path)
        for subfolder in subfolders:
            subfolder_path = os.path.join(old_scen_folder_path, subfolder)
            print(subfolder_path)
            if os.path.exists(subfolder_path) and os.path.isdir(subfolder_path):
                for filename in os.listdir(subfolder_path):
                    file_path = os.path.join(subfolder_path, filename)
                    # Check if it's a file
                    if os.path.isfile(file_path):
                        print(filename)
                        new_filename=filename.replace(folder_name,"")
                        new_filename=new_filename[1:]
                        print(new_filename)
                        print(file_path)
                        new_file_path=os.path.join(folder_path,new_filename)
                        print(new_file_path)
                        shutil.copy(file_path,new_file_path)
        #shutil.move(old_scen_folder_path, folder_path)
    else:
        print(f"No matching .map file found in folder: {folder_name}")

