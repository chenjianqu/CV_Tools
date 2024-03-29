/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#include "io_utils.h"


/**
 * 清除某个目录下的所有文件
 * @param path
 */
void ClearDirectory(const string &path){
    fs::path dir_path(path);
    if(!fs::exists(dir_path)){
        int isCreate = mkdir(path.c_str(),S_IRUSR | S_IWUSR | S_IXUSR | S_IRWXG | S_IRWXO);
    }
    fs::directory_iterator dir_iter(dir_path);
    for(auto &it : dir_iter){
        remove(it.path().c_str());
    }
}

/**
 * 获取目录下的所有文件名
 * @param path
 * @return
 */
vector<fs::path> GetDirectoryFileNames(const string &path){
    vector<fs::path> names;
    fs::path dir_path(path);
    if(!fs::exists(dir_path))
        return {};
    fs::directory_iterator dir_iter(dir_path);
    for(auto &it : dir_iter)
        names.emplace_back(it.path().filename());
    std::sort(names.begin(),names.end());
    return names;
}


/**
 * 检查一个路径是否是目录
 * @param dir
 * @return
 */
bool CheckIsDir(const string &dir) {
    if (! std::filesystem::exists(dir)) {
        cout<<dir<<" not exists. Please check."<<endl;
        return false;
    }
    std::filesystem::directory_entry entry(dir);
    if (entry.is_directory())
        return true;
    return false;
}

/**
 * 递归的搜索一个目录下所有的图像文件，以 jpg,jpeg,png 结尾的文件
 * @param dir
 * @param files
 */
void GetAllImageFiles(const string& dir, vector<string> &files) {
    // 首先检查目录是否为空，以及是否是目录
    if (!CheckIsDir(dir))
        return;

    // 递归遍历所有的文件
    std::filesystem::directory_iterator iters(dir);
    for(auto &iter: iters) {
        string file_path(dir);
        file_path += "/";
        file_path += iter.path().filename();

        // 查看是否是目录，如果是目录则循环递归
        if (CheckIsDir(file_path)) {
            GetAllImageFiles(file_path, files);
        }
        //不是目录则检查后缀是否是图像
        else {
            string extension = iter.path().extension(); // 获取文件的后缀名
            if (extension == ".jpg" || extension == ".png" || extension == ".jpeg") {
                files.push_back(file_path);
            }
        }
    }
}

