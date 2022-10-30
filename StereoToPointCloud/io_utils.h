/*******************************************************
 * Copyright (C) 2022, Chen Jianqu, Shanghai University
 *
 * This file is part of dynamic_vins.
 * Github:https://github.com/chenjianqu/dynamic_vins
 *
 * Licensed under the MIT License;
 * you may not use this file except in compliance with the License.
 *******************************************************/

#ifndef DYNAMIC_VINS_IO_UTILS_H
#define DYNAMIC_VINS_IO_UTILS_H

#include <filesystem>


#include "def.h"




void ClearDirectory(const string &path);

vector<fs::path> GetDirectoryFileNames(const string &path);

bool CheckIsDir(const string &dir);

void GetAllImageFiles(const string& dir, vector<string> &files) ;


cv::Scalar BgrColor(const string &color_str,bool is_norm=true);




#endif //DYNAMIC_VINS_IO_UTILS_H
