#include <../include/file_manager.hpp>
#include <boost/filesystem.hpp>
#include <glog/logging.h>

bool FileManager::CreateFile(std::ofstream& ofs, std::string file_path){
    ofs.close();
    boost::filesystem::remove(file_path.c_str());

    ofs.open(file_path.c_str(), std::ios::out);
    if(!ofs){
        LOG(WARNING)<<"无法生成文件："<<std::endl << file_path << std::endl;
        return false;
    }

    return true;
}

bool FileManager::InitDirectory(std::string directory_path, std::string use_for){
    if(boost::filesystem::is_directory(directory_path)){                    //是否存在目录
        boost::filesystem::remove_all(directory_path);                      //删除目录，包括文件
    }

    return CreateDirectory(directory_path, use_for);
}

bool FileManager::CreateDirectory(std::string directory_path, std::string use_for){
    if(!boost::filesystem::is_directory(directory_path)){
        boost::filesystem::create_directory(directory_path);
    }
    if(!boost::filesystem::is_directory(directory_path)){
        LOG(WARNING) << "CANNOT create directory" << std::endl << directory_path << std::endl;
        return false;
    }
    return true;
}
