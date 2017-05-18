#ifndef UTILS_H_
#define UTILS_H_

#include <iostream>
#include <string>
// Boost headers
#include <boost/filesystem.hpp>

class Utils
{
public:
	//静态成员函数在使用的使用，可以直接将类Utils当做namespace，不必定义一个Utils类型的对象
	//用法：Utils::get_clouds_filenames(...)
	static bool pcd_ex;
	static bool ply_ex;
	static void get_clouds_filenames
		(
		const std::string & pDirectoryPath,
		std::vector<std::string> & pFilenames
		);

};

#endif