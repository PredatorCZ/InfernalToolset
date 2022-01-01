/*  ZAPExtract
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
    ".zap$",
    {},
};

struct ZAPExtract : ReflectorBase<ZAPExtract> {
} settings;

REFLECT(CLASS(ZAPExtract));

AppInfo_s appInfo{
    AppInfo_s::CONTEXT_VERSION,
    AppMode_e::EXTRACT,
    ArchiveLoadType::FILTERED,
    ZAPExtract_DESC " v" ZAPExtract_VERSION ", " ZAPExtract_COPYRIGHT
                    "Lukas Cone",
    reinterpret_cast<ReflectorFriend *>(&settings),
    filters,
};

const AppInfo_s *AppInitModule() { return &appInfo; }

struct Header {
  static constexpr uint32 TAG = 0x20031031;
  uint32 tag;
  uint32 version;
  uint32 dataSize;
  uint32 rootOffset;
  uint32 totalFiles;
};

struct File {
  std::string name;
  uint32 offset;
  uint32 ucompSize;
  uint32 compSize;
};

struct Tree {
  std::vector<File> files;
  std::map<std::string, Tree> children;
};

void LoadTree(BinReaderRef rd, Tree &parent) {
  uint16 numFolders;
  uint16 numFiles;
  uint32 filesOffset;
  rd.Read(numFolders);
  rd.Read(numFiles);
  rd.Read(filesOffset);

  if (numFolders) {
    std::vector<uint32> offsets;
    const size_t localOffset = rd.Tell();
    rd.ReadContainer(offsets, numFolders);

    for (auto o : offsets) {
      rd.Seek(localOffset + o);
      std::string dirName;
      rd.ReadContainer<uint8>(dirName);
      uint32 treeOffset;
      rd.Read(treeOffset);
      rd.Seek(treeOffset);
      Tree child;
      LoadTree(rd, child);
      parent.children.emplace(std::move(dirName), std::move(child));
    }
  }

  if (numFiles) {
    rd.Seek(filesOffset);
    std::vector<uint32> offsets;
    const size_t localOffset = rd.Tell();
    rd.ReadContainer(offsets, numFiles);

    for (auto o : offsets) {
      rd.Seek(localOffset + o);
      File cFile;
      rd.ReadContainer<uint8>(cFile.name);
      rd.Read(cFile.offset);
      rd.Read(cFile.ucompSize);
      rd.Read(cFile.compSize);
      uint32 check;
      rd.Read(check);
      rd.Read(check);
      rd.Read(check);
      parent.files.emplace_back(std::move(cFile));
    }
  }
}

void GenerateFolders(const Tree &root, std::string path,
                     AppExtractContext &ctx) {
  for (auto &[name, child] : root.children) {
    auto cName = path + name;
    ctx.AddFolderPath(cName);
    GenerateFolders(child, cName + "/", ctx);
  }
}

void ExtractFiles(BinReaderRef rd, const Tree &tree, std::string path,
                  AppExtractContext &ctx) {
  for (auto &[name, child] : tree.children) {
    ExtractFiles(rd, child, path + name + "/", ctx);
  }

  for (auto &f : tree.files) {
    rd.Seek(f.offset);
    ctx.NewFile(path + f.name);
    std::string inBuffer;
    rd.ReadContainer(inBuffer, f.compSize);

    if (f.compSize == f.ucompSize) {
      ctx.SendData(inBuffer);
      continue;
    }

    std::string outBuffer;
    outBuffer.resize(f.ucompSize);

    ProcessZAPStream(inBuffer, outBuffer, f.compSize);

    ctx.SendData(outBuffer);
  }
}

void AppExtractFile(std::istream &stream, AppExtractContext *ctx) {
  BinReaderRef rd(stream);
  const size_t fileSize = rd.GetSize();
  rd.Seek(fileSize - 20);
  Header hdr;
  rd.Read(hdr);

  if (hdr.tag != Header::TAG) {
    throw es::InvalidHeaderError(hdr.tag);
  }

  rd.Seek(hdr.rootOffset);
  Tree tree;
  LoadTree(rd, tree);

  if (ctx->RequiresFolders()) {
    GenerateFolders(tree, "", *ctx);
    ctx->GenerateFolders();
  }

  ExtractFiles(rd, tree, "", *ctx);
}
