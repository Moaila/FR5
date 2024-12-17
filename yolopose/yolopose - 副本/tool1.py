import os
import shutil


def rename_and_move_files(src_folder, dest_folder):
    # 确保目标文件夹存在，如果不存在则创建
    if not os.path.exists(dest_folder):
        os.makedirs(dest_folder)

    # 遍历源文件夹中的所有文件
    for filename in os.listdir(src_folder):
        # 检查文件名是否以数字开头
        if filename.split('.')[0].isdigit():
            # 提取文件名中的数字部分
            number = int(filename.split('.')[0])
            # 计算新的文件名
            new_number = number + 501
            # 构建新的文件名
            new_filename = f"{new_number}.{'.'.join(filename.split('.')[1:])}"
            # 获取文件的完整路径
            old_file_path = os.path.join(src_folder, filename)
            new_file_path = os.path.join(dest_folder, new_filename)
            # 移动并重命名文件
            shutil.move(old_file_path, new_file_path)
            print(f"Moved and renamed: {filename} -> {new_filename}")


# 使用示例
src_folder = "./shaoyigelei"  # 将此路径替换为源文件夹路径
dest_folder = "./shaoyigelei"  # 将此路径替换为目标文件夹路径
rename_and_move_files(src_folder, dest_folder)
