#include "stereo_utility/config.hpp"


M210_STEREO::Config* M210_STEREO::Config::single_instance_ = NULL;

M210_STEREO::Config::Config()
{
}

M210_STEREO::Config::~Config()
{
    if (file_.isOpened())
    {
        file_.release();
    }
}

M210_STEREO::Config&
M210_STEREO::Config::instance()
{
    return *Config::single_instance_;
}

M210_STEREO::Config*
M210_STEREO::Config::instancePtr()
{
    return Config::single_instance_;
}

void M210_STEREO::Config::setParamFile(const std::string& file_name)
{
    if(!Config::single_instance_)
    {
        Config::single_instance_ = new Config();
    }

    Config::instancePtr()->file_ = cv::FileStorage( file_name, cv::FileStorage::READ );

    if(!Config::instancePtr()->file_.isOpened())
    {
        std::cerr << "Failed to open " << file_name << " file\n";
        Config::instancePtr()->file_.release();
    }
}
