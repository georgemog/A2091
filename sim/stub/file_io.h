#pragma once
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
typedef int64_t __off64_t;
struct fileTYPE {
	FILE *f = nullptr;
	int64_t size = 0;
	char name[1024] = {};
	bool opened() const { return f != nullptr; }
};
int FileOpenEx(fileTYPE *file, const char *name, int mode, char mute = 0, int use_zip = 1);
int FileClose(fileTYPE *file);
int FileSeek(fileTYPE *file, __off64_t offset, int origin);
int FileReadAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int FileWriteAdv(fileTYPE *file, void *pBuffer, int length, int failres = 0);
int FileExists(const char *name, int use_zip = 1);
int FileCanWrite(const char *name);
int FileLoad(const char *name, void *pBuffer, int size);
const char *getRootDir();
