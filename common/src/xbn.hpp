/*  Infernal XBN format
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

#pragma once
#include "datas/binreader_stream.hpp"
#include "datas/matrix44.hpp"
#include "datas/vectors.hpp"

struct Header {
  static constexpr uint32 VERSION = 17;
  uint32 version;
  std::string name;
  uint32 numVerts;
  uint32 numFaces;
  uint32 numNodes;
  uint32 numTextures;
  bool useUV2;

  void Read(BinReaderRef rd) {
    rd.Read(version);

    if (version != Header::VERSION) {
      throw es::InvalidVersionError(version);
    }

    rd.ReadContainer(name);
    rd.Read(numVerts);
    rd.Read(numFaces);
    rd.Read(numNodes);
    rd.Read(numTextures);
    rd.Read(useUV2);
  }
};

struct Vertex {
  Vector pos;
  Vector normal;
  Vector2 uv;
};

struct PhysType {
  uint32 unk;
  std::string name;

  void Read(BinReaderRef rd) {
    rd.Read(unk);
    rd.ReadContainer(name);
  }
};

struct Bone {
  uint8 unk0;
  uint8 unk1;
  int8 skinBoneId;
  int8 parentId;
  std::string boneName;
  es::Matrix44 transform;
  es::Matrix44 ibm;
  es::Matrix44 mtx2;

  void Read(BinReaderRef rd) {
    rd.Read(unk0);
    rd.ReadContainer(boneName);
    rd.Read(parentId);
    rd.Read(transform);
    rd.Read(ibm);
    rd.Read(mtx2);
    rd.Read(unk1);
    rd.Read(skinBoneId);

    std::transform(boneName.begin(), boneName.end(), boneName.begin(),
                   [](unsigned char c) { return std::tolower(c); });
  }
};

struct Material {
  std::string textureName;
  std::string flags;
  std::string shaderName;
  uint32 endFace;
  char unkData[6];

  void Read(BinReaderRef rd) {
    rd.ReadContainer(textureName);
    rd.Read(endFace);
    rd.Read(unkData);
    rd.ReadContainer(flags);
    rd.ReadContainer(shaderName);
  }
};
