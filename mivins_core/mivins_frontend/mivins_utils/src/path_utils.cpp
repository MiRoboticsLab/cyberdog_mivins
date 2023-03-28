// Copyright (c) 2023 Beijing Xiaomi Mobile Software Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <mivins/utils/path_utils.h>
#include <dirent.h>
#include <sys/stat.h>

#include <glog/logging.h>

namespace mivins
{
    void ConcatenateFolderAndFileName(
        const std::string &folder, const std::string &file_name,
        std::string *path)
    {
        CHECK_NOTNULL(path);
        CHECK(!file_name.empty());
        *path = folder;
        if (path->back() != '/')
        {
            *path += '/';
        }
        *path = *path + file_name;
    }

    std::string ConcatenateFolderAndFileName(
        const std::string &folder, const std::string &file_name)
    {
        std::string path;
        ConcatenateFolderAndFileName(folder, file_name, &path);
        return path;
    }

    bool FileExists(const std::string &path)
    {
        struct stat st;
        return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFREG);
    }

    bool PathExists(const std::string &path)
    {
        struct stat st;
        return stat(path.c_str(), &st) == 0 && (st.st_mode & S_IFDIR);
    }

    void SplitPathAndFilename(
        const std::string &str, std::string *path, std::string *filename)
    {
        CHECK_NOTNULL(path)->clear();
        CHECK_NOTNULL(filename)->clear();
        const size_t right_delim = str.find_last_of("/");
        if (right_delim != std::string::npos)
        {
            *path = str.substr(0, right_delim);
        }
        *filename = str.substr(right_delim + 1);
    }

    // Returns full paths. No recursion.
    void GetFilesAndSubfolders(const std::string &path,
                               std::vector<std::string> *files,
                               std::vector<std::string> *folders)
    {
        CHECK_NOTNULL(files)->clear();
        CHECK_NOTNULL(folders)->clear();
        CHECK(PathExists(path));

        DIR *directory_stream = CHECK_NOTNULL(opendir(path.c_str()));
        struct dirent *directory_entry;
        while ((directory_entry = readdir(directory_stream)) != NULL)
        {
            const std::string filename(directory_entry->d_name);
            if ((filename == ".") || (filename == ".."))
            {
                continue;
            }
            const std::string abs_path = path + "/" + filename;

            if (FileExists(abs_path))
            {
                files->emplace_back(abs_path);
            }
            else
            {
                CHECK(PathExists(abs_path));
                folders->emplace_back(abs_path);
            }
        }

        closedir(directory_stream);
    }

    std::string getBaseName(const std::string &filename)
    {
        const std::string separator = "/";
        std::size_t last_separator = filename.find_last_of(separator);
        if (last_separator == std::string::npos)
        {
            return std::string();
        }
        return filename.substr(0, last_separator);
    }
} // namespace mivins