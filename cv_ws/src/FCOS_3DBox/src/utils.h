//
// Created by chen on 2022/5/2.
//

#ifndef FCOS_3DBOX_UTILS_H
#define FCOS_3DBOX_UTILS_H

#include <regex>
#include <string>
#include <chrono>
#include <filesystem>

void split(const std::string& source, std::vector<std::string>& tokens, const std::string& delimiters = " ") {
    std::regex re(delimiters);
    std::copy(std::sregex_token_iterator(source.begin(), source.end(),re,-1),
              std::sregex_token_iterator(),
              std::back_inserter(tokens));
}


class TicToc{
public:
    TicToc(){
        Tic();
    }
    void Tic(){
        start_ = std::chrono::system_clock::now();
    }
    double Toc(){
        end_ = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_ - start_;
        return elapsed_seconds.count() * 1000;
    }
    double TocThenTic(){
        auto t= Toc();
        Tic();
        return t;
    }
    void TocPrintTic(const char* str){
        std::cout << str << ":" << Toc() << " ms" <<std:: endl;
        Tic();
    }
private:
    std::chrono::time_point<std::chrono::system_clock> start_, end_;
};



// 检查一个路径是否是目录
bool checkIsDir(const std::string &dir) {
    if (! std::filesystem::exists(dir)) {
        std::cout<<dir<<" not exists. Please check."<<std::endl;
        return false;
    }
    std::filesystem::directory_entry entry(dir);
    if (entry.is_directory())
        return true;
    return false;
}


#endif //FCOS_3DBOX_UTILS_H
