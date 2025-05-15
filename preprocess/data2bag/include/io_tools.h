// Copyright 2022 HX. All Rights Reserved.
// Author:  Jiagang Chen (jiagang.chen@rhino.auto)

#pragma once

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

namespace hx_slam {

namespace io {

inline std::vector<std::string> StringSplit(const std::string &str,
                                            const std::string &delim) {
  std::vector<std::string> elems;
  boost::split(elems, str, boost::is_any_of(delim), boost::token_compress_on);
  return elems;
}

inline void StringToLower(std::string *str) {
  std::transform(str->begin(), str->end(), str->begin(), ::tolower);
}

inline std::string StringReplace(const std::string &str,
                                 const std::string &old_str,
                                 const std::string &new_str) {
  if (old_str.empty()) {
    return str;
  }
  size_t position = 0;
  std::string mod_str = str;
  while ((position = mod_str.find(old_str, position)) != std::string::npos) {
    mod_str.replace(position, old_str.size(), new_str);
    position += new_str.size();
  }
  return mod_str;
}

enum class CopyType { COPY, HARD_LINK, SOFT_LINK };

// Append trailing slash to string if it does not yet end with a slash.
inline std::string EnsureTrailingSlash(const std::string &str) {
  if (str.length() > 0) {
    if (str.back() != '/') {
      return str + "/";
    }
  } else {
    return str + "/";
  }
  return str;
}

// Check whether file name has the file extension (case insensitive).
inline bool HasFileExtension(const std::string &file_name,
                             const std::string &ext) {
  if (ext.empty()) return false;
  if (ext.at(0) == '.') return false;
  std::string ext_lower = ext;
  StringToLower(&ext_lower);
  if (file_name.size() >= ext_lower.size() &&
      file_name.substr(file_name.size() - ext_lower.size(), ext_lower.size()) ==
          ext_lower) {
    return true;
  }
  return false;
}

// Split the path into its root and extension, for example,
// "dir/file.jpg" into "dir/file" and ".jpg".
inline void SplitFileExtension(const std::string &path, std::string *root,
                               std::string *ext) {
  const auto parts = StringSplit(path, ".");
  assert(parts.size() != 0);
  if (parts.size() == 1) {
    *root = parts[0];
    *ext = "";
  } else {
    *root = "";
    for (size_t i = 0; i < parts.size() - 1; ++i) {
      *root += parts[i] + ".";
    }
    *root = root->substr(0, root->length() - 1);
    if (parts.back() == "") {
      *ext = "";
    } else {
      *ext = "." + parts.back();
    }
  }
}

// Copy or link file from source to destination path
inline void FileCopy(const std::string &src_path, const std::string &dst_path,
                     CopyType type = CopyType::COPY) {
  switch (type) {
    case CopyType::COPY:
      boost::filesystem::copy_file(src_path, dst_path);
      break;
    case CopyType::HARD_LINK:
      boost::filesystem::create_hard_link(src_path, dst_path);
      break;
    case CopyType::SOFT_LINK:
      boost::filesystem::create_symlink(src_path, dst_path);
      break;
  }
}

// Check if the path points to an existing directory.
inline bool ExistsFile(const std::string &path) {
  return boost::filesystem::is_regular_file(path);
}

// Check if the path points to an existing directory.
inline bool ExistsDir(const std::string &path) {
  return boost::filesystem::is_directory(path);
}

// Check if the path points to an existing file or directory.
inline bool ExistsPath(const std::string &path) {
  return boost::filesystem::exists(path);
}

// Delete the directory if it exists.
inline void DeleteDirIfExists(const std::string &path) {
  if (ExistsDir(path)) {
    boost::filesystem::remove_all(path);
  }
}

// Create the directory if it does not exist.
inline void CreateDirIfNotExists(const std::string &path) {
  if (!ExistsDir(path)) {
    boost::filesystem::create_directory(path);
  }
}

// Extract the base name of a path, e.g., "image.jpg" for "/dir/image.jpg".
inline std::string GetPathBaseName(const std::string &path) {
  const std::vector<std::string> names =
      StringSplit(StringReplace(path, "\\", "/"), "/");
  if (names.size() > 1 && names.back() == "") {
    return names[names.size() - 2];
  } else {
    return names.back();
  }
}

// Get the path of the parent directory for the given path.
inline std::string GetParentDir(const std::string &path) {
  return boost::filesystem::path(path).parent_path().string();
}

// Get the relative path between from and to. Both the from and to paths must
// exist.
inline std::string GetRelativePath(const std::string &from,
                                   const std::string &to) {
  // This implementation is adapted from:
  // https://stackoverflow.com/questions/10167382
  // A native implementation in boost::filesystem is only available starting
  // from boost version 1.60.
  using namespace boost::filesystem;

  path from_path = canonical(path(from));
  path to_path = canonical(path(to));

  // Start at the root path and while they are the same then do nothing then
  // when they first diverge take the entire from path, swap it with '..'
  // segments, and then append the remainder of the to path.
  path::const_iterator from_iter = from_path.begin();
  path::const_iterator to_iter = to_path.begin();

  // Loop through both while they are the same to find nearest common directory
  while (from_iter != from_path.end() && to_iter != to_path.end() &&
         (*to_iter) == (*from_iter)) {
    ++to_iter;
    ++from_iter;
  }

  // Replace from path segments with '..' (from => nearest common directory)
  path rel_path;
  while (from_iter != from_path.end()) {
    rel_path /= "..";
    ++from_iter;
  }

  // Append the remainder of the to path (nearest common directory => to)
  while (to_iter != to_path.end()) {
    rel_path /= *to_iter;
    ++to_iter;
  }

  return rel_path.string();
}

// Join multiple paths into one path.
template <typename... T>
inline std::string JoinPaths(T const &...paths) {
  boost::filesystem::path result;
  int unpack[]{0, (result = result / boost::filesystem::path(paths), 0)...};
  static_cast<void>(unpack);
  return result.string();
}

// Return list of files in directory.
inline std::vector<std::string> GetFileList(const std::string &path) {
  std::vector<std::string> file_list;
  for (auto it = boost::filesystem::directory_iterator(path);
       it != boost::filesystem::directory_iterator(); ++it) {
    if (boost::filesystem::is_regular_file(*it)) {
      const boost::filesystem::path file_path = *it;
      file_list.push_back(file_path.string());
    }
  }
  std::sort(file_list.begin(), file_list.end());
  return file_list;
}

// Return list of files, recursively in all sub-directories.
inline std::vector<std::string> GetRecursiveFileList(const std::string &path) {
  std::vector<std::string> file_list;
  for (auto it = boost::filesystem::recursive_directory_iterator(path);
       it != boost::filesystem::recursive_directory_iterator(); ++it) {
    if (boost::filesystem::is_regular_file(*it)) {
      const boost::filesystem::path file_path = *it;
      file_list.push_back(file_path.string());
    }
  }
  return file_list;
}

// Return list of directories, recursively in all sub-directories.
inline std::vector<std::string> GetDirList(const std::string &path) {
  std::vector<std::string> dir_list;
  for (auto it = boost::filesystem::directory_iterator(path);
       it != boost::filesystem::directory_iterator(); ++it) {
    if (boost::filesystem::is_directory(*it)) {
      const boost::filesystem::path dir_path = *it;
      dir_list.push_back(dir_path.string());
    }
  }
  return dir_list;
}

// Return list of directories, recursively in all sub-directories.
inline std::vector<std::string> GetRecursiveDirList(const std::string &path) {
  std::vector<std::string> dir_list;
  for (auto it = boost::filesystem::recursive_directory_iterator(path);
       it != boost::filesystem::recursive_directory_iterator(); ++it) {
    if (boost::filesystem::is_directory(*it)) {
      const boost::filesystem::path dir_path = *it;
      dir_list.push_back(dir_path.string());
    }
  }
  return dir_list;
}

}  // namespace io
}  // namespace hx_slam