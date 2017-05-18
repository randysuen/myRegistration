#include "utils.h"
//读取一个目录下所有点云文件名
//pDirectoryPath是路径名, pFilenames是该路径下的所有文件名
void Utils::get_clouds_filenames (const std::string & pDirectoryPath, std::vector<std::string> & pFilenames)
{
	//将string转化为boost文件类型
	boost::filesystem::path directory(pDirectoryPath);
	pFilenames.clear();
	//判断路径是否存在，以及是否是目录
	if (!boost::filesystem::exists(directory)||!boost::filesystem::is_directory(directory)){
		std::cerr << "invalid directory!" << std::endl;
	}
	std::vector<boost::filesystem::path> paths;
	//directory_iterator是目录下所有文件（文件夹）的迭代器
	//第一项是迭代器的起点（第一个文件），第二项是迭代器的终点（无参数）
	std::copy(boost::filesystem::directory_iterator(directory), boost::filesystem::directory_iterator(), std::back_inserter(paths));
	std::sort(paths.begin(), paths.end());
	//const_iterator不能改变其指向的元素
	for (std::vector<boost::filesystem::path>::const_iterator it = paths.begin(); it != paths.end(); ++it)
	{
		if (it->extension().string() == ".pcd" || it->extension().string() == ".ply")
		{
			std::cout << *it << "\n";
			pFilenames.push_back(it->relative_path().string());
		}
	}
	pcd_ex = paths.begin()->extension().string() == ".pcd";
	ply_ex = paths.begin()->extension().string() == ".ply";

}