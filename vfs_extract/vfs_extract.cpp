/*  VFSExtract
    Copyright(C) 2022 Lukas Cone

    This program is free software : you can redistribute it and / or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.If not, see <https://www.gnu.org/licenses/>.
*/

#include "datas/app_context.hpp"
#include "datas/binreader_stream.hpp"
#include "datas/except.hpp"
#include "datas/reflector.hpp"
#include "project.h"
#include "zlib.hpp"
#include <vector>

es::string_view filters[]{
    ".vfs$",
    {},
};

struct VFSExtract : ReflectorBase<VFSExtract> {
} settings;

REFLECT(CLASS(VFSExtract));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::EXTRACT,
    ArchiveLoadType::FILTERED,
    VFSExtract_DESC " v" VFSExtract_VERSION ", " VFSExtract_COPYRIGHT
                    "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

struct Folder {
  uint32 crc;
  uint32 id;
  int32 parentId;
  int32 firstChildId;
  int32 numFiles;
};

struct FolderEx : Folder {
  uint32 numChildren;
};

struct File {
  uint32 crc;
  uint32 id;
  int32 mode;
  int32 folderId;
  int32 dataOffset;
  int32 dataSize;
};

struct VFS2 {
  static constexpr uint32 ID = CompileFourCC("VFS2");
  std::vector<FolderEx> folders;
  std::vector<File> files;
  std::vector<std::string> fileNames;
  std::vector<std::string> folderNames;
  size_t dataOffset;

  void Read(BinReaderRef rd) {
    uint32 id;
    rd.Read(id);

    if (id != ID) {
      throw es::InvalidHeaderError(id);
    }

    rd.ReadContainerLambda(folders, [&](BinReaderRef rd, FolderEx &item) {
      rd.Read<Folder>(item);

      if (item.parentId >= 0) {
        folders.at(item.parentId).numChildren++;
      }
    });
    rd.ReadContainer(files);
    uint32 namesOffset;
    rd.Read(namesOffset);
    dataOffset = rd.Tell();
    rd.Seek(namesOffset);

    rd.ReadContainerLambda(
        fileNames, [](BinReaderRef rd, auto &item) { rd.ReadContainer(item); });
    rd.ReadContainerLambda(folderNames, [](BinReaderRef rd, auto &item) {
      rd.ReadContainer(item);
    });
  }

  void GenerateFolders(const FolderEx &item, std::string path,
                       AppExtractContext &ctx) {
    for (size_t i = 0; i < item.numChildren; i++) {
      auto &child = folders.at(item.firstChildId + i);
      auto cName = path + folderNames.at(child.id);
      ctx.AddFolderPath(cName);
      GenerateFolders(child, cName + "/", ctx);
    }
  }

  void GenerateFolders(AppExtractContext &ctx) {
    GenerateFolders(folders.front(), "", ctx);
  }

  std::string MakePath(const FolderEx &item, std::string path) {
    if (item.parentId >= 0) {
      path = folderNames.at(item.id) + "/" + path;
      return MakePath(folders.at(item.parentId), path);
    }

    return path;
  }

  void ExtractFiles(BinReaderRef rd, AppExtractContext &ctx) {
    for (auto &f : files) {
      auto &fName = fileNames.at(f.id);
      auto &parent = folders.at(f.folderId);
      auto path = MakePath(parent, "");
      ctx.NewFile(path + fName);
      rd.Seek(dataOffset + f.dataOffset);

      if (f.mode) {
        const size_t beginStuff = rd.Tell();
        uint32 ucompsize;
        uint32 dataOffset;
        rd.Read(ucompsize);
        rd.Read(dataOffset);
        const size_t numChunks = (dataOffset - 8) / 4;
        std::vector<uint32> chunks;
        rd.ReadContainer(chunks, numChunks);
        std::string inBuffer;
        std::string outBuffer;
        outBuffer.resize(ucompsize);

        {
          size_t lastOffset = dataOffset;

          for (auto &c : chunks) {
            auto tmp = c;
            c -= lastOffset;
            lastOffset = tmp;
          }

          chunks.push_back(f.dataSize - lastOffset);
        }

        for (auto c : chunks) {
          rd.ReadContainer(inBuffer, c);

          size_t totalOut = ProcessVFSStream(inBuffer, outBuffer, c);

          ctx.SendData({outBuffer.data(), totalOut});
        }
      } else {
        std::string inBuffer;
        rd.ReadContainer(inBuffer, f.dataSize);
        ctx.SendData(inBuffer);
      }
    }
  }
};

void AppExtractFile(std::istream &stream, AppExtractContext *ctx) {
  BinReaderRef rd(stream);
  VFS2 main;
  rd.Read(main);

  if (ctx->RequiresFolders()) {
    main.GenerateFolders(*ctx);
    ctx->GenerateFolders();
  }

  main.ExtractFiles(rd, *ctx);
}
