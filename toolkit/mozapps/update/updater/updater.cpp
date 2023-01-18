/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/. */

/**
 *  Manifest Format
 *  ---------------
 *
 *  contents = 1*( line )
 *  line     = method LWS *( param LWS ) CRLF
 *  CRLF     = "\r\n"
 *  LWS      = 1*( " " | "\t" )
 *
 *  Available methods for the manifest file:
 *
 *  updatev2.manifest
 *  -----------------
 *  method   = "add" | "add-if" | "patch" | "patch-if" | "remove" |
 *             "rmdir" | "rmrfdir" | type
 *
 *  'type' is the update type (e.g. complete or partial) and when present MUST
 *  be the first entry in the update manifest. The type is used to support
 *  downgrades by causing the actions defined in precomplete to be performed.
 *
 *  updatev3.manifest
 *  -----------------
 *  method   = "add" | "add-if" | "add-if-not" | "patch" | "patch-if" |
 *             "remove" | "rmdir" | "rmrfdir" | type
 *
 *  'add-if-not' adds a file if it doesn't exist.
 *
 *  precomplete
 *  -----------
 *  method   = "remove" | "rmdir"
 */
#include "bspatch.h"
#include "progressui.h"
#include "archivereader.h"
#include "readstrings.h"
#include "errors.h"
#include "bzlib.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <limits.h>
#include <errno.h>
#include <algorithm>

#include "updatelogging.h"

#include "mozilla/Compiler.h"
#include "mozilla/Types.h"

// Amount of the progress bar to use in each of the 3 update stages,
// should total 100.0.
#define PROGRESS_PREPARE_SIZE 20.0f
#define PROGRESS_EXECUTE_SIZE 75.0f
#define PROGRESS_FINISH_SIZE   5.0f

// Amount of time in ms to wait for the parent process to close
#ifdef DEBUG
// Use a large value for debug builds since the xpcshell tests take a long time.
#define PARENT_WAIT 30000
#else
#define PARENT_WAIT 10000
#endif
#define IMMERSIVE_PARENT_WAIT 15000

#if defined(XP_MACOSX)
// These functions are defined in launchchild_osx.mm
void LaunchChild(int argc, char **argv);
void LaunchMacPostProcess(const char* aAppBundle);
#endif

#ifndef _O_BINARY
# define _O_BINARY 0
#endif

#ifndef NULL
# define NULL (0)
#endif

#ifndef SSIZE_MAX
# define SSIZE_MAX LONG_MAX
#endif

// We want to use execv to invoke the callback executable on platforms where
// we were launched using execv.  See nsUpdateDriver.cpp.
#if defined(XP_UNIX) && !defined(XP_MACOSX)
#define USE_EXECV
#endif

#if defined(MOZ_WIDGET_GONK)
# include "automounter_gonk.h"
# include <unistd.h>
# include <android/log.h>
# include <linux/ioprio.h>
# include <sys/resource.h>

#if ANDROID_VERSION < 21
// The only header file in bionic which has a function prototype for ioprio_set
// is libc/include/sys/linux-unistd.h. However, linux-unistd.h conflicts
// badly with unistd.h, so we declare the prototype for ioprio_set directly.
extern "C" MOZ_EXPORT int ioprio_set(int which, int who, int ioprio);
#else
# include <sys/syscall.h>
static int ioprio_set(int which, int who, int ioprio) {
      return syscall(__NR_ioprio_set, which, who, ioprio);
}
#endif

# define MAYBE_USE_HARD_LINKS 1
static bool sUseHardLinks = true;
#else
# define MAYBE_USE_HARD_LINKS 0
#endif

#ifdef XP_WIN
#include "registrycertificates.h"
BOOL PathAppendSafe(LPWSTR base, LPCWSTR extra);
BOOL PathGetSiblingFilePath(LPWSTR destinationBuffer,
                            LPCWSTR siblingFilePath,
                            LPCWSTR newFileName);
#include "updatehelper.h"

// Closes the handle if valid and if the updater is elevated returns with the
// return code specified. This prevents multiple launches of the callback
// application by preventing the elevated process from launching the callback.
#define EXIT_WHEN_ELEVATED(path, handle, retCode) \
  { \
      if (handle != INVALID_HANDLE_VALUE) { \
        CloseHandle(handle); \
      } \
      if (_waccess(path, F_OK) == 0 && NS_tremove(path) != 0) { \
        LogFinish(); \
        return retCode; \
      } \
  }
#endif

//-----------------------------------------------------------------------------

// This variable lives in libbz2.  It's declared in bzlib_private.h, so we just
// declare it here to avoid including that entire header file.
#define BZ2_CRC32TABLE_UNDECLARED

#if MOZ_IS_GCC
extern "C"  __attribute__((visibility("default"))) unsigned int BZ2_crc32Table[256];
#undef BZ2_CRC32TABLE_UNDECLARED
#elif defined(__SUNPRO_C) || defined(__SUNPRO_CC)
extern "C" __global unsigned int BZ2_crc32Table[256];
#undef BZ2_CRC32TABLE_UNDECLARED
#endif
#if defined(BZ2_CRC32TABLE_UNDECLARED)
extern "C" unsigned int BZ2_crc32Table[256];
#undef BZ2_CRC32TABLE_UNDECLARED
#endif

static unsigned int
crc32(const unsigned char *buf, unsigned int len)
{
  unsigned int crc = 0xffffffffL;

  const unsigned char *end = buf + len;
  for (; buf != end; ++buf)
    crc = (crc << 8) ^ BZ2_crc32Table[(crc >> 24) ^ *buf];

  crc = ~crc;
  return crc;
}

//-----------------------------------------------------------------------------

// A simple stack based container for a FILE struct that closes the
// file descriptor from its destructor.
class AutoFile
{
public:
  explicit AutoFile(FILE* file = nullptr)
    : mFile(file) {
  }

  ~AutoFile() {
    if (mFile != nullptr)
      fclose(mFile);
  }

  AutoFile &operator=(FILE* file) {
    if (mFile != 0)
      fclose(mFile);
    mFile = file;
    return *this;
  }

  operator FILE*() {
    return mFile;
  }

  FILE* get() {
    return mFile;
  }

private:
  FILE* mFile;
};

struct MARChannelStringTable {
  MARChannelStringTable()
  {
    MARChannelID[0] = '\0';
  }

  char MARChannelID[MAX_TEXT_LEN];
};

//-----------------------------------------------------------------------------

typedef void (* ThreadFunc)(void *param);

#ifdef XP_WIN
#include <process.h>

class Thread
{
public:
  int Run(ThreadFunc func, void *param)
  {
    mThreadFunc = func;
    mThreadParam = param;

    unsigned int threadID;

    mThread = (HANDLE) _beginthreadex(nullptr, 0, ThreadMain, this, 0,
                                      &threadID);

    return mThread ? 0 : -1;
  }
  int Join()
  {
    WaitForSingleObject(mThread, INFINITE);
    CloseHandle(mThread);
    return 0;
  }
private:
  static unsigned __stdcall ThreadMain(void *p)
  {
    Thread *self = (Thread *) p;
    self->mThreadFunc(self->mThreadParam);
    return 0;
  }
  HANDLE     mThread;
  ThreadFunc mThreadFunc;
  void      *mThreadParam;
};

#elif defined(XP_UNIX)
#include <pthread.h>

class Thread
{
public:
  int Run(ThreadFunc func, void *param)
  {
    return pthread_create(&thr, nullptr, (void* (*)(void *)) func, param);
  }
  int Join()
  {
    void *result;
    return pthread_join(thr, &result);
  }
private:
  pthread_t thr;
};

#else
#error "Unsupported platform"
#endif

//-----------------------------------------------------------------------------

static NS_tchar* gPatchDirPath;
static NS_tchar gInstallDirPath[MAXPATHLEN];
static NS_tchar gWorkingDirPath[MAXPATHLEN];
static ArchiveReader gArchiveReader;
static bool gSucceeded = false;
static bool sStagedUpdate = false;
static bool sReplaceRequest = false;
static bool sUsingService = false;
static bool sIsOSUpdate = false;

#ifdef XP_WIN
// The current working directory specified in the command line.
static NS_tchar* gDestPath;
static NS_tchar gCallbackRelPath[MAXPATHLEN];
static NS_tchar gCallbackBackupPath[MAXPATHLEN];
#endif

static const NS_tchar kWhitespace[] = NS_T(" \t");
static const NS_tchar kNL[] = NS_T("\r\n");
static const NS_tchar kQuote[] = NS_T("\"");

static inline size_t
mmin(size_t a, size_t b)
{
  return (a > b) ? b : a;
}

static NS_tchar*
mstrtok(const NS_tchar *delims, NS_tchar **str)
{
  if (!*str || !**str)
    return nullptr;

  // skip leading "whitespace"
  NS_tchar *ret = *str;
  const NS_tchar *d;
  do {
    for (d = delims; *d != NS_T('\0'); ++d) {
      if (*ret == *d) {
        ++ret;
        break;
      }
    }
  } while (*d);

  if (!*ret) {
    *str = ret;
    return nullptr;
  }

  NS_tchar *i = ret;
  do {
    for (d = delims; *d != NS_T('\0'); ++d) {
      if (*i == *d) {
        *i = NS_T('\0');
        *str = ++i;
        return ret;
      }
    }
    ++i;
  } while (*i);

  *str = nullptr;
  return ret;
}

#ifdef XP_WIN
/**
 * Coverts a relative update path to a full path for Windows.
 *
 * @param  relpath
 *         The relative path to convert to a full path.
 * @return valid filesystem full path or nullptr if memory allocation fails.
 */
static NS_tchar*
get_full_path(const NS_tchar *relpath)
{
  size_t lendestpath = NS_tstrlen(gDestPath);
  size_t lenrelpath = NS_tstrlen(relpath);
  NS_tchar *s = (NS_tchar *) malloc((lendestpath + lenrelpath + 1) * sizeof(NS_tchar));
  if (!s)
    return nullptr;

  NS_tchar *c = s;

  NS_tstrcpy(c, gDestPath);
  c += lendestpath;
  NS_tstrcat(c, relpath);
  c += lenrelpath;
  *c = NS_T('\0');
  c++;
  return s;
}
#endif

/**
 * Gets the platform specific path and performs simple checks to the path. If
 * the path checks don't pass nullptr will be returned.
 *
 * @param  line
 *         The line from the manifest that contains the path.
 * @param  isdir
 *         Whether the path is a directory path. Defaults to false.
 * @return valid filesystem path or nullptr if the path checks fail.
 */
static NS_tchar*
get_valid_path(NS_tchar **line, bool isdir = false)
{
  NS_tchar *path = mstrtok(kQuote, line);
  if (!path) {
    LOG(("get_valid_path: unable to determine path: " LOG_S, line));
    return nullptr;
  }

  // All paths must be relative from the current working directory
  if (path[0] == NS_T('/')) {
    LOG(("get_valid_path: path must be relative: " LOG_S, path));
    return nullptr;
  }

#ifdef XP_WIN
  // All paths must be relative from the current working directory
  if (path[0] == NS_T('\\') || path[1] == NS_T(':')) {
    LOG(("get_valid_path: path must be relative: " LOG_S, path));
    return nullptr;
  }
#endif

  if (isdir) {
    // Directory paths must have a trailing forward slash.
    if (path[NS_tstrlen(path) - 1] != NS_T('/')) {
      LOG(("get_valid_path: directory paths must have a trailing forward " \
           "slash: " LOG_S, path));
      return nullptr;
    }

    // Remove the trailing forward slash because stat on Windows will return
    // ENOENT if the path has a trailing slash.
    path[NS_tstrlen(path) - 1] = NS_T('\0');
  }

  // Don't allow relative paths that resolve to a parent directory.
  if (NS_tstrstr(path, NS_T("..")) != nullptr) {
    LOG(("get_valid_path: paths must not contain '..': " LOG_S, path));
    return nullptr;
  }

  return path;
}

static NS_tchar*
get_quoted_path(const NS_tchar *path)
{
  size_t lenQuote = NS_tstrlen(kQuote);
  size_t lenPath = NS_tstrlen(path);
  size_t len = lenQuote + lenPath + lenQuote + 1;

  NS_tchar *s = (NS_tchar *) malloc(len * sizeof(NS_tchar));
  if (!s)
    return nullptr;

  NS_tchar *c = s;
  NS_tstrcpy(c, kQuote);
  c += lenQuote;
  NS_tstrcat(c, path);
  c += lenPath;
  NS_tstrcat(c, kQuote);
  c += lenQuote;
  *c = NS_T('\0');
  c++;
  return s;
}

static void ensure_write_permissions(const NS_tchar *path)
{
#ifdef XP_WIN
  (void) _wchmod(path, _S_IREAD | _S_IWRITE);
#else
  struct stat fs;
  if (!stat(path, &fs) && !(fs.st_mode & S_IWUSR)) {
    (void)chmod(path, fs.st_mode | S_IWUSR);
  }
#endif
}

static int ensure_remove(const NS_tchar *path)
{
  ensure_write_permissions(path);
  int rv = NS_tremove(path);
  if (rv)
    LOG(("ensure_remove: failed to remove file: " LOG_S ", rv: %d, err: %d",
         path, rv, errno));
  return rv;
}

// Remove the directory pointed to by path and all of its files and sub-directories.
static int ensure_remove_recursive(const NS_tchar *path,
                                   bool continueEnumOnFailure = false)
{
  // We use lstat rather than stat here so that we can successfully remove
  // symlinks.
  struct NS_tstat_t sInfo;
  int rv = NS_tlstat(path, &sInfo);
  if (rv) {
    // This error is benign
    return rv;
  }
  if (!S_ISDIR(sInfo.st_mode)) {
    return ensure_remove(path);
  }

  NS_tDIR *dir;
  NS_tdirent *entry;

  dir = NS_topendir(path);
  if (!dir) {
    LOG(("ensure_remove_recursive: unable to open directory: " LOG_S
         ", rv: %d, err: %d", path, rv, errno));
    return rv;
  }

  while ((entry = NS_treaddir(dir)) != 0) {
    if (NS_tstrcmp(entry->d_name, NS_T(".")) &&
        NS_tstrcmp(entry->d_name, NS_T(".."))) {
      NS_tchar childPath[MAXPATHLEN];
      NS_tsnprintf(childPath, sizeof(childPath)/sizeof(childPath[0]),
                   NS_T("%s/%s"), path, entry->d_name);
      rv = ensure_remove_recursive(childPath);
      if (rv && !continueEnumOnFailure) {
        break;
      }
    }
  }

  NS_tclosedir(dir);

  if (rv == OK) {
    ensure_write_permissions(path);
    rv = NS_trmdir(path);
    if (rv) {
      LOG(("ensure_remove_recursive: unable to remove directory: " LOG_S
           ", rv: %d, err: %d", path, rv, errno));
    }
  }
  return rv;
}

static bool is_read_only(const NS_tchar *flags)
{
  size_t length = NS_tstrlen(flags);
  if (length == 0)
    return false;

  // Make sure the string begins with "r"
  if (flags[0] != NS_T('r'))
    return false;

  // Look for "r+" or "r+b"
  if (length > 1 && flags[1] == NS_T('+'))
    return false;

  // Look for "rb+"
  if (NS_tstrcmp(flags, NS_T("rb+")) == 0)
    return false;

  return true;
}

static FILE* ensure_open(const NS_tchar *path, const NS_tchar *flags, unsigned int options)
{
  ensure_write_permissions(path);
  FILE* f = NS_tfopen(path, flags);
  if (is_read_only(flags)) {
    // Don't attempt to modify the file permissions if the file is being opened
    // in read-only mode.
    return f;
  }
  if (NS_tchmod(path, options) != 0) {
    if (f != nullptr) {
      fclose(f);
    }
    return nullptr;
  }
  struct NS_tstat_t ss;
  if (NS_tstat(path, &ss) != 0 || ss.st_mode != options) {
    if (f != nullptr) {
      fclose(f);
    }
    return nullptr;
  }
  return f;
}

// Ensure that the directory containing this file exists.
static int ensure_parent_dir(const NS_tchar *path)
{
  int rv = OK;

  NS_tchar *slash = (NS_tchar *) NS_tstrrchr(path, NS_T('/'));
  if (slash) {
    *slash = NS_T('\0');
    rv = ensure_parent_dir(path);
    // Only attempt to create the directory if we're not at the root
    if (rv == OK && *path) {
      rv = NS_tmkdir(path, 0755);
      // If the directory already exists, then ignore the error.
      if (rv < 0 && errno != EEXIST) {
        LOG(("ensure_parent_dir: failed to create directory: " LOG_S ", " \
             "err: %d", path, errno));
        rv = WRITE_ERROR;
      } else {
        rv = OK;
      }
    }
    *slash = NS_T('/');
  }
  return rv;
}

#ifdef XP_UNIX
static int ensure_copy_symlink(const NS_tchar *path, const NS_tchar *dest)
{
  // Copy symlinks by creating a new symlink to the same target
  NS_tchar target[MAXPATHLEN + 1] = {NS_T('\0')};
  int rv = readlink(path, target, MAXPATHLEN);
  if (rv == -1) {
    LOG(("ensure_copy_symlink: failed to read the link: " LOG_S ", err: %d",
         path, errno));
    return READ_ERROR;
  }
  rv = symlink(target, dest);
  if (rv == -1) {
    LOG(("ensure_copy_symlink: failed to create the new link: " LOG_S ", target: " LOG_S " err: %d",
         dest, target, errno));
    return READ_ERROR;
  }
  return 0;
}
#endif

#if MAYBE_USE_HARD_LINKS
/*
 * Creates a hardlink (destFilename) which points to the existing file
 * (srcFilename).
 *
 * @return 0 if successful, an error otherwise
 */

static int
create_hard_link(const NS_tchar *srcFilename, const NS_tchar *destFilename)
{
  if (link(srcFilename, destFilename) < 0) {
    LOG(("link(%s, %s) failed errno = %d", srcFilename, destFilename, errno));
    return WRITE_ERROR;
  }
  return OK;
}
#endif

// Copy the file named path onto a new file named dest.
static int ensure_copy(const NS_tchar *path, const NS_tchar *dest)
{
#ifdef XP_WIN
  // Fast path for Windows
  bool result = CopyFileW(path, dest, false);
  if (!result) {
    LOG(("ensure_copy: failed to copy the file " LOG_S " over to " LOG_S ", lasterr: %x",
         path, dest, GetLastError()));
    return WRITE_ERROR;
  }
  return 0;
#else
  struct NS_tstat_t ss;
  int rv = NS_tlstat(path, &ss);
  if (rv) {
    LOG(("ensure_copy: failed to read file status info: " LOG_S ", err: %d",
         path, errno));
    return READ_ERROR;
  }

#ifdef XP_UNIX
  if (S_ISLNK(ss.st_mode)) {
    return ensure_copy_symlink(path, dest);
  }
#endif

#if MAYBE_USE_HARD_LINKS
  if (sUseHardLinks) {
    if (!create_hard_link(path, dest)) {
      return OK;
    }
    // Since we failed to create the hard link, fall through and copy the file.
    sUseHardLinks = false;
  }
#endif

  AutoFile infile(ensure_open(path, NS_T("rb"), ss.st_mode));
  if (!infile) {
    LOG(("ensure_copy: failed to open the file for reading: " LOG_S ", err: %d",
         path, errno));
    return READ_ERROR;
  }
  AutoFile outfile(ensure_open(dest, NS_T("wb"), ss.st_mode));
  if (!outfile) {
    LOG(("ensure_copy: failed to open the file for writing: " LOG_S ", err: %d",
         dest, errno));
    return WRITE_ERROR;
  }

  // This block size was chosen pretty arbitrarily but seems like a reasonable
  // compromise. For example, the optimal block size on a modern OS X machine
  // is 100k */
  const int blockSize = 32 * 1024;
  void* buffer = malloc(blockSize);
  if (!buffer)
    return UPDATER_MEM_ERROR;

  while (!feof(infile.get())) {
    size_t read = fread(buffer, 1, blockSize, infile);
    if (ferror(infile.get())) {
      LOG(("ensure_copy: failed to read the file: " LOG_S ", err: %d",
           path, errno));
      free(buffer);
      return READ_ERROR;
    }

    size_t written = 0;

    while (written < read) {
      size_t chunkWritten = fwrite(buffer, 1, read - written, outfile);
      if (chunkWritten <= 0) {
        LOG(("ensure_copy: failed to write the file: " LOG_S ", err: %d",
             dest, errno));
        free(buffer);
        return WRITE_ERROR;
      }

      written += chunkWritten;
    }
  }

  rv = NS_tchmod(dest, ss.st_mode);

  free(buffer);
  return rv;
#endif
}

template <unsigned N>
struct copy_recursive_skiplist {
  NS_tchar paths[N][MAXPATHLEN];

  void append(unsigned index, const NS_tchar *path, const NS_tchar *suffix) {
    NS_tsnprintf(paths[index], MAXPATHLEN, NS_T("%s/%s"), path, suffix);
  }

  bool find(const NS_tchar *path) {
    for (int i = 0; i < static_cast<int>(N); ++i) {
      if (!NS_tstricmp(paths[i], path)) {
        return true;
      }
    }
    return false;
  }
};

// Copy all of the files and subdirectories under path to a new directory named dest.
// The path names in the skiplist will be skipped and will not be copied.
template <unsigned N>
static int ensure_copy_recursive(const NS_tchar *path, const NS_tchar *dest,
                                 copy_recursive_skiplist<N>& skiplist)
{
  struct NS_tstat_t sInfo;
  int rv = NS_tlstat(path, &sInfo);
  if (rv) {
    LOG(("ensure_copy_recursive: path doesn't exist: " LOG_S ", rv: %d, err: %d",
         path, rv, errno));
    return READ_ERROR;
  }

#ifdef XP_UNIX
  if (S_ISLNK(sInfo.st_mode)) {
    return ensure_copy_symlink(path, dest);
  }
#endif

  if (!S_ISDIR(sInfo.st_mode)) {
    return ensure_copy(path, dest);
  }

  rv = NS_tmkdir(dest, sInfo.st_mode);
  if (rv < 0 && errno != EEXIST) {
    LOG(("ensure_copy_recursive: could not create destination directory: " LOG_S ", rv: %d, err: %d",
         path, rv, errno));
    return WRITE_ERROR;
  }

  NS_tDIR *dir;
  NS_tdirent *entry;

  dir = NS_topendir(path);
  if (!dir) {
    LOG(("ensure_copy_recursive: path is not a directory: " LOG_S ", rv: %d, err: %d",
         path, rv, errno));
    return READ_ERROR;
  }

  while ((entry = NS_treaddir(dir)) != 0) {
    if (NS_tstrcmp(entry->d_name, NS_T(".")) &&
        NS_tstrcmp(entry->d_name, NS_T(".."))) {
      NS_tchar childPath[MAXPATHLEN];
      NS_tsnprintf(childPath, sizeof(childPath)/sizeof(childPath[0]),
                   NS_T("%s/%s"), path, entry->d_name);
      if (skiplist.find(childPath)) {
        continue;
      }
      NS_tchar childPathDest[MAXPATHLEN];
      NS_tsnprintf(childPathDest, sizeof(childPathDest)/sizeof(childPathDest[0]),
                   NS_T("%s/%s"), dest, entry->d_name);
      rv = ensure_copy_recursive(childPath, childPathDest, skiplist);
      if (rv) {
        break;
      }
    }
  }
  NS_tclosedir(dir);
  return rv;
}

// Renames the specified file to the new file specified. If the destination file
// exists it is removed.
static int rename_file(const NS_tchar *spath, const NS_tchar *dpath,
                       bool allowDirs = false)
{
  int rv = ensure_parent_dir(dpath);
  if (rv)
    return rv;

  struct NS_tstat_t spathInfo;
  rv = NS_tstat(spath, &spathInfo);
  if (rv) {
    LOG(("rename_file: failed to read file status info: " LOG_S ", " \
         "err: %d", spath, errno));
    return READ_ERROR;
  }

  if (!S_ISREG(spathInfo.st_mode)) {
    if (allowDirs && !S_ISDIR(spathInfo.st_mode)) {
      LOG(("rename_file: path present, but not a file: " LOG_S ", err: %d",
           spath, errno));
      return UNEXPECTED_FILE_OPERATION_ERROR;
    } else {
      LOG(("rename_file: proceeding to rename the directory"));
    }
  }

  if (!NS_taccess(dpath, F_OK)) {
    if (ensure_remove(dpath)) {
      LOG(("rename_file: destination file exists and could not be " \
           "removed: " LOG_S, dpath));
      return WRITE_ERROR;
    }
  }

  if (NS_trename(spath, dpath) != 0) {
    LOG(("rename_file: failed to rename file - src: " LOG_S ", " \
         "dst:" LOG_S ", err: %d", spath, dpath, errno));
    return WRITE_ERROR;
  }

  return OK;
}

#ifdef XP_WIN
// Remove the directory pointed to by path and all of its files and
// sub-directories. If a file is in use move it to the tobedeleted directory
// and attempt to schedule removal of the file on reboot
static int remove_recursive_on_reboot(const NS_tchar *path, const NS_tchar *deleteDir)
{
  struct NS_tstat_t sInfo;
  int rv = NS_tlstat(path, &sInfo);
  if (rv) {
    // This error is benign
    return rv;
  }

  if (!S_ISDIR(sInfo.st_mode)) {
    NS_tchar tmpDeleteFile[MAXPATHLEN];
    GetTempFileNameW(deleteDir, L"rep", 0, tmpDeleteFile);
    NS_tremove(tmpDeleteFile);
    rv = rename_file(path, tmpDeleteFile, false);
    if (MoveFileEx(rv ? path : tmpDeleteFile, nullptr, MOVEFILE_DELAY_UNTIL_REBOOT)) {
      LOG(("remove_recursive_on_reboot: file will be removed on OS reboot: "
           LOG_S, rv ? path : tmpDeleteFile));
    } else {
      LOG(("remove_recursive_on_reboot: failed to schedule OS reboot removal of "
           "file: " LOG_S, rv ? path : tmpDeleteFile));
    }
    return rv;
  }

  NS_tDIR *dir;
  NS_tdirent *entry;

  dir = NS_topendir(path);
  if (!dir) {
    LOG(("remove_recursive_on_reboot: unable to open directory: " LOG_S
         ", rv: %d, err: %d",
         path, rv, errno));
    return rv;
  }

  while ((entry = NS_treaddir(dir)) != 0) {
    if (NS_tstrcmp(entry->d_name, NS_T(".")) &&
        NS_tstrcmp(entry->d_name, NS_T(".."))) {
      NS_tchar childPath[MAXPATHLEN];
      NS_tsnprintf(childPath, sizeof(childPath)/sizeof(childPath[0]),
                   NS_T("%s/%s"), path, entry->d_name);
      // There is no need to check the return value of this call since this
      // function is only called after an update is successful and there is not
      // much that can be done to recover if it isn't successful. There is also
      // no need to log the value since it will have already been logged.
      remove_recursive_on_reboot(childPath, deleteDir);
    }
  }

  NS_tclosedir(dir);

  if (rv == OK) {
    ensure_write_permissions(path);
    rv = NS_trmdir(path);
    if (rv) {
      LOG(("remove_recursive_on_reboot: unable to remove directory: " LOG_S
           ", rv: %d, err: %d", path, rv, errno));
    }
  }
  return rv;
}
#endif

//-----------------------------------------------------------------------------

// Create a backup of the specified file by renaming it.
static int backup_create(const NS_tchar *path)
{
  NS_tchar backup[MAXPATHLEN];
  NS_tsnprintf(backup, sizeof(backup)/sizeof(backup[0]),
               NS_T("%s") BACKUP_EXT, path);

  return rename_file(path, backup);
}

// Rename the backup of the specified file that was created by renaming it back
// to the original file.
static int backup_restore(const NS_tchar *path)
{
  NS_tchar backup[MAXPATHLEN];
  NS_tsnprintf(backup, sizeof(backup)/sizeof(backup[0]),
               NS_T("%s") BACKUP_EXT, path);

  if (NS_taccess(backup, F_OK)) {
    LOG(("backup_restore: backup file doesn't exist: " LOG_S, backup));
    return OK;
  }

  return rename_file(backup, path);
}

// Discard the backup of the specified file that was created by renaming it.
static int backup_discard(const NS_tchar *path)
{
  NS_tchar backup[MAXPATHLEN];
  NS_tsnprintf(backup, sizeof(backup)/sizeof(backup[0]),
               NS_T("%s") BACKUP_EXT, path);

  // Nothing to discard
  if (NS_taccess(backup, F_OK)) {
    return OK;
  }

  int rv = ensure_remove(backup);
#if defined(XP_WIN)
  if (rv && !sStagedUpdate && !sReplaceRequest) {
    LOG(("backup_discard: unable to remove: " LOG_S, backup));
    NS_tchar path[MAXPATHLEN];
    GetTempFileNameW(DELETE_DIR, L"moz", 0, path);
    if (rename_file(backup, path)) {
      LOG(("backup_discard: failed to rename file:" LOG_S ", dst:" LOG_S,
           backup, path));
      return WRITE_ERROR;
    }
    // The MoveFileEx call to remove the file on OS reboot will fail if the
    // process doesn't have write access to the HKEY_LOCAL_MACHINE registry key
    // but this is ok since the installer / uninstaller will delete the
    // directory containing the file along with its contents after an update is
    // applied, on reinstall, and on uninstall.
    if (MoveFileEx(path, nullptr, MOVEFILE_DELAY_UNTIL_REBOOT)) {
      LOG(("backup_discard: file renamed and will be removed on OS " \
           "reboot: " LOG_S, path));
    } else {
      LOG(("backup_discard: failed to schedule OS reboot removal of " \
           "file: " LOG_S, path));
    }
  }
#else
  if (rv)
    return WRITE_ERROR;
#endif

  return OK;
}

// Helper function for post-processing a temporary backup.
static void backup_finish(const NS_tchar *path, int status)
{
  if (status == OK)
    backup_discard(path);
  else
    backup_restore(path);
}

//-----------------------------------------------------------------------------

static int DoUpdate();

class Action
{
public:
  Action() : mProgressCost(1), mNext(nullptr) { }
  virtual ~Action() { }

  virtual int Parse(NS_tchar *line) = 0;

  // Do any preprocessing to ensure that the action can be performed.  Execute
  // will be called if this Action and all others return OK from this method.
  virtual int Prepare() = 0;

  // Perform the operation.  Return OK to indicate success.  After all actions
  // have been executed, Finish will be called.  A requirement of Execute is
  // that its operation be reversable from Finish.
  virtual int Execute() = 0;

  // Finish is called after execution of all actions.  If status is OK, then
  // all actions were successfully executed.  Otherwise, some action failed.
  virtual void Finish(int status) = 0;

  int mProgressCost;
private:
  Action* mNext;

  friend class ActionList;
};

class RemoveFile : public Action
{
public:
  RemoveFile() : mFile(nullptr), mSkip(0) { }

  int Parse(NS_tchar *line);
  int Prepare();
  int Execute();
  void Finish(int status);

private:
  const NS_tchar *mFile;
  int mSkip;
};

int
RemoveFile::Parse(NS_tchar *line)
{
  // format "<deadfile>"

  mFile = get_valid_path(&line);
  if (!mFile)
    return PARSE_ERROR;

  return OK;
}

int
RemoveFile::Prepare()
{
  // Skip the file if it already doesn't exist.
  int rv = NS_taccess(mFile, F_OK);
  if (rv) {
    mSkip = 1;
    mProgressCost = 0;
    return OK;
  }

  LOG(("PREPARE REMOVEFILE " LOG_S, mFile));

  // Make sure that we're actually a file...
  struct NS_tstat_t fileInfo;
  rv = NS_tstat(mFile, &fileInfo);
  if (rv) {
    LOG(("failed to read file status info: " LOG_S ", err: %d", mFile,
         errno));
    return READ_ERROR;
  }

  if (!S_ISREG(fileInfo.st_mode)) {
    LOG(("path present, but not a file: " LOG_S, mFile));
    return UNEXPECTED_FILE_OPERATION_ERROR;
  }

  NS_tchar *slash = (NS_tchar *) NS_tstrrchr(mFile, NS_T('/'));
  if (slash) {
    *slash = NS_T('\0');
    rv = NS_taccess(mFile, W_OK);
    *slash = NS_T('/');
  } else {
    rv = NS_taccess(NS_T("."), W_OK);
  }

  if (rv) {
    LOG(("access failed: %d", errno));
    return WRITE_ERROR;
  }

  return OK;
}

int
RemoveFile::Execute()
{
  if (mSkip)
    return OK;

  LOG(("EXECUTE REMOVEFILE " LOG_S, mFile));

  // The file is checked for existence here and in Prepare since it might have
  // been removed by a separate instruction: bug 311099.
  int rv = NS_taccess(mFile, F_OK);
  if (rv) {
    LOG(("file cannot be removed because it does not exist; skipping"));
    mSkip = 1;
    return OK;
  }

  // Rename the old file. It will be removed in Finish.
  rv = backup_create(mFile);
  if (rv) {
    LOG(("backup_create failed: %d", rv));
    return rv;
  }

  return OK;
}

void
RemoveFile::Finish(int status)
{
  if (mSkip)
    return;

  LOG(("FINISH REMOVEFILE " LOG_S, mFile));

  backup_finish(mFile, status);
}

class RemoveDir : public Action
{
public:
  RemoveDir() : mDir(nullptr), mSkip(0) { }

  virtual int Parse(NS_tchar *line);
  virtual int Prepare(); // check that the source dir exists
  virtual int Execute();
  virtual void Finish(int status);

private:
  const NS_tchar *mDir;
  int mSkip;
};

int
RemoveDir::Parse(NS_tchar *line)
{
  // format "<deaddir>/"

  mDir = get_valid_path(&line, true);
  if (!mDir)
    return PARSE_ERROR;

  return OK;
}

int
RemoveDir::Prepare()
{
  // We expect the directory to exist if we are to remove it.
  int rv = NS_taccess(mDir, F_OK);
  if (rv) {
    mSkip = 1;
    mProgressCost = 0;
    return OK;
  }

  LOG(("PREPARE REMOVEDIR " LOG_S "/", mDir));

  // Make sure that we're actually a dir.
  struct NS_tstat_t dirInfo;
  rv = NS_tstat(mDir, &dirInfo);
  if (rv) {
    LOG(("failed to read directory status info: " LOG_S ", err: %d", mDir,
         errno));
    return READ_ERROR;
  }

  if (!S_ISDIR(dirInfo.st_mode)) {
    LOG(("path present, but not a directory: " LOG_S, mDir));
    return UNEXPECTED_FILE_OPERATION_ERROR;
  }

  rv = NS_taccess(mDir, W_OK);
  if (rv) {
    LOG(("access failed: %d, %d", rv, errno));
    return WRITE_ERROR;
  }

  return OK;
}

int
RemoveDir::Execute()
{
  if (mSkip)
    return OK;

  LOG(("EXECUTE REMOVEDIR " LOG_S "/", mDir));

  // The directory is checked for existence at every step since it might have
  // been removed by a separate instruction: bug 311099.
  int rv = NS_taccess(mDir, F_OK);
  if (rv) {
    LOG(("directory no longer exists; skipping"));
    mSkip = 1;
  }

  return OK;
}

void
RemoveDir::Finish(int status)
{
  if (mSkip || status != OK)
    return;

  LOG(("FINISH REMOVEDIR " LOG_S "/", mDir));

  // The directory is checked for existence at every step since it might have
  // been removed by a separate instruction: bug 311099.
  int rv = NS_taccess(mDir, F_OK);
  if (rv) {
    LOG(("directory no longer exists; skipping"));
    return;
  }


  if (status == OK) {
    if (NS_trmdir(mDir)) {
      LOG(("non-fatal error removing directory: " LOG_S "/, rv: %d, err: %d",
           mDir, rv, errno));
    }
  }
}

class AddFile : public Action
{
public:
  AddFile() : mFile(nullptr)
            , mAdded(false)
            { }

  virtual int Parse(NS_tchar *line);
  virtual int Prepare();
  virtual int Execute();
  virtual void Finish(int status);

private:
  const NS_tchar *mFile;
  bool mAdded;
};

int
AddFile::Parse(NS_tchar *line)
{
  // format "<newfile>"

  mFile = get_valid_path(&line);
  if (!mFile)
    return PARSE_ERROR;

  return OK;
}

int
AddFile::Prepare()
{
  LOG(("PREPARE ADD " LOG_S, mFile));

  return OK;
}

int
AddFile::Execute()
{
  LOG(("EXECUTE ADD " LOG_S, mFile));

  int rv;

  // First make sure that we can actually get rid of any existing file.
  rv = NS_taccess(mFile, F_OK);
  if (rv == 0) {
    rv = backup_create(mFile);
    if (rv)
      return rv;
  } else {
    rv = ensure_parent_dir(mFile);
    if (rv)
      return rv;
  }

#ifdef XP_WIN
  char sourcefile[MAXPATHLEN];
  if (!WideCharToMultiByte(CP_UTF8, 0, mFile, -1, sourcefile, MAXPATHLEN,
                           nullptr, nullptr)) {
    LOG(("error converting wchar to utf8: %d", GetLastError()));
    return STRING_CONVERSION_ERROR;
  }

  rv = gArchiveReader.ExtractFile(sourcefile, mFile);
#else
  rv = gArchiveReader.ExtractFile(mFile, mFile);
#endif
  if (!rv) {
    mAdded = true;
  }
  return rv;
}

void
AddFile::Finish(int status)
{
  LOG(("FINISH ADD " LOG_S, mFile));
  // When there is an update failure and a file has been added it is removed
  // here since there might not be a backup to replace it.
  if (status && mAdded)
    NS_tremove(mFile);
  backup_finish(mFile, status);
}

class PatchFile : public Action
{
public:
  PatchFile() : mPatchIndex(-1), buf(nullptr) { }

  virtual ~PatchFile();

  virtual int Parse(NS_tchar *line);
  virtual int Prepare(); // should check for patch file and for checksum here
  virtual int Execute();
  virtual void Finish(int status);

private:
  int LoadSourceFile(FILE* ofile);

  static int sPatchIndex;

  const NS_tchar *mPatchFile;
  const NS_tchar *mFile;
  int mPatchIndex;
  MBSPatchHeader header;
  unsigned char *buf;
  NS_tchar spath[MAXPATHLEN];
};

int PatchFile::sPatchIndex = 0;

PatchFile::~PatchFile()
{
  // delete the temporary patch file
  if (spath[0])
    NS_tremove(spath);

  if (buf)
    free(buf);
}

int
PatchFile::LoadSourceFile(FILE* ofile)
{
  struct stat os;
  int rv = fstat(fileno((FILE *)ofile), &os);
  if (rv) {
    LOG(("LoadSourceFile: unable to stat destination file: " LOG_S ", " \
         "err: %d", mFile, errno));
    return READ_ERROR;
  }

  if (uint32_t(os.st_size) != header.slen) {
    LOG(("LoadSourceFile: destination file size %d does not match expected size %d",
         uint32_t(os.st_size), header.slen));
    return UNEXPECTED_FILE_OPERATION_ERROR;
  }

  buf = (unsigned char *) malloc(header.slen);
  if (!buf)
    return UPDATER_MEM_ERROR;

  size_t r = header.slen;
  unsigned char *rb = buf;
  while (r) {
    const size_t count = mmin(SSIZE_MAX, r);
    size_t c = fread(rb, 1, count, ofile);
    if (c != count) {
      LOG(("LoadSourceFile: error reading destination file: " LOG_S,
           mFile));
      return READ_ERROR;
    }

    r -= c;
    rb += c;
  }

  // Verify that the contents of the source file correspond to what we expect.

  unsigned int crc = crc32(buf, header.slen);

  if (crc != header.scrc32) {
    LOG(("LoadSourceFile: destination file crc %d does not match expected " \
         "crc %d", crc, header.scrc32));
    return CRC_ERROR;
  }

  return OK;
}

int
PatchFile::Parse(NS_tchar *line)
{
  // format "<patchfile>" "<filetopatch>"

  // Get the path to the patch file inside of the mar
  mPatchFile = mstrtok(kQuote, &line);
  if (!mPatchFile)
    return PARSE_ERROR;

  // consume whitespace between args
  NS_tchar *q = mstrtok(kQuote, &line);
  if (!q)
    return PARSE_ERROR;

  mFile = get_valid_path(&line);
  if (!mFile)
    return PARSE_ERROR;

  return OK;
}

int
PatchFile::Prepare()
{
  LOG(("PREPARE PATCH " LOG_S, mFile));

  // extract the patch to a temporary file
  mPatchIndex = sPatchIndex++;

  NS_tsnprintf(spath, sizeof(spath)/sizeof(spath[0]),
               NS_T("%s/updating/%d.patch"), gWorkingDirPath, mPatchIndex);

  NS_tremove(spath);

  FILE *fp = NS_tfopen(spath, NS_T("wb"));
  if (!fp)
    return WRITE_ERROR;

#ifdef XP_WIN
  char sourcefile[MAXPATHLEN];
  if (!WideCharToMultiByte(CP_UTF8, 0, mPatchFile, -1, sourcefile, MAXPATHLEN,
                           nullptr, nullptr)) {
    LOG(("error converting wchar to utf8: %d", GetLastError()));
    return STRING_CONVERSION_ERROR;
  }

  int rv = gArchiveReader.ExtractFileToStream(sourcefile, fp);
#else
  int rv = gArchiveReader.ExtractFileToStream(mPatchFile, fp);
#endif
  fclose(fp);
  return rv;
}

int
PatchFile::Execute()
{
  LOG(("EXECUTE PATCH " LOG_S, mFile));

  AutoFile pfile(NS_tfopen(spath, NS_T("rb")));
  if (pfile == nullptr)
    return READ_ERROR;

  int rv = MBS_ReadHeader(pfile, &header);
  if (rv)
    return rv;

  FILE *origfile = nullptr;
#ifdef XP_WIN
  if (NS_tstrcmp(mFile, gCallbackRelPath) == 0) {
    // Read from the copy of the callback when patching since the callback can't
    // be opened for reading to prevent the application from being launched.
    origfile = NS_tfopen(gCallbackBackupPath, NS_T("rb"));
  } else {
    origfile = NS_tfopen(mFile, NS_T("rb"));
  }
#else
  origfile = NS_tfopen(mFile, NS_T("rb"));
#endif

  if (!origfile) {
    LOG(("unable to open destination file: " LOG_S ", err: %d", mFile,
         errno));
    return READ_ERROR;
  }

  rv = LoadSourceFile(origfile);
  fclose(origfile);
  if (rv) {
    LOG(("LoadSourceFile failed"));
    return rv;
  }

  // Rename the destination file if it exists before proceeding so it can be
  // used to restore the file to its original state if there is an error.
  struct NS_tstat_t ss;
  rv = NS_tstat(mFile, &ss);
  if (rv) {
    LOG(("failed to read file status info: " LOG_S ", err: %d", mFile,
         errno));
    return READ_ERROR;
  }

  rv = backup_create(mFile);
  if (rv)
    return rv;

#if defined(HAVE_POSIX_FALLOCATE)
  AutoFile ofile(ensure_open(mFile, NS_T("wb+"), ss.st_mode));
  posix_fallocate(fileno((FILE *)ofile), 0, header.dlen);
#elif defined(XP_WIN)
  bool shouldTruncate = true;
  // Creating the file, setting the size, and then closing the file handle
  // lessens fragmentation more than any other method tested. Other methods that
  // have been tested are:
  // 1. _chsize / _chsize_s reduced fragmentation but though not completely.
  // 2. _get_osfhandle and then setting the size reduced fragmentation though
  //    not completely. There are also reports of _get_osfhandle failing on
  //    mingw.
  HANDLE hfile = CreateFileW(mFile,
                             GENERIC_WRITE,
                             0,
                             nullptr,
                             CREATE_ALWAYS,
                             FILE_ATTRIBUTE_NORMAL,
                             nullptr);

  if (hfile != INVALID_HANDLE_VALUE) {
    if (SetFilePointer(hfile, header.dlen,
                       nullptr, FILE_BEGIN) != INVALID_SET_FILE_POINTER &&
        SetEndOfFile(hfile) != 0) {
      shouldTruncate = false;
    }
    CloseHandle(hfile);
  }

  AutoFile ofile(ensure_open(mFile, shouldTruncate ? NS_T("wb+") : NS_T("rb+"), ss.st_mode));
#elif defined(XP_MACOSX)
  AutoFile ofile(ensure_open(mFile, NS_T("wb+"), ss.st_mode));
  // Modified code from FileUtils.cpp
  fstore_t store = {F_ALLOCATECONTIG, F_PEOFPOSMODE, 0, header.dlen};
  // Try to get a continous chunk of disk space
  rv = fcntl(fileno((FILE *)ofile), F_PREALLOCATE, &store);
  if (rv == -1) {
    // OK, perhaps we are too fragmented, allocate non-continuous
    store.fst_flags = F_ALLOCATEALL;
    rv = fcntl(fileno((FILE *)ofile), F_PREALLOCATE, &store);
  }

  if (rv != -1) {
    ftruncate(fileno((FILE *)ofile), header.dlen);
  }
#else
  AutoFile ofile(ensure_open(mFile, NS_T("wb+"), ss.st_mode));
#endif

  if (ofile == nullptr) {
    LOG(("unable to create new file: " LOG_S ", err: %d", mFile, errno));
    return WRITE_ERROR;
  }

#ifdef XP_WIN
  if (!shouldTruncate) {
    fseek(ofile, 0, SEEK_SET);
  }
#endif

  rv = MBS_ApplyPatch(&header, pfile, buf, ofile);

  // Go ahead and do a bit of cleanup now to minimize runtime overhead.
  // Set pfile to nullptr to make AutoFile close the file so it can be deleted
  // on Windows.
  pfile = nullptr;
  NS_tremove(spath);
  spath[0] = NS_T('\0');
  free(buf);
  buf = nullptr;

  return rv;
}

void
PatchFile::Finish(int status)
{
  LOG(("FINISH PATCH " LOG_S, mFile));

  backup_finish(mFile, status);
}

class AddIfFile : public AddFile
{
public:
  AddIfFile() : mTestFile(nullptr) { }

  virtual int Parse(NS_tchar *line);
  virtual int Prepare();
  virtual int Execute();
  virtual void Finish(int status);

protected:
  const NS_tchar *mTestFile;
};

int
AddIfFile::Parse(NS_tchar *line)
{
  // format "<testfile>" "<newfile>"

  mTestFile = get_valid_path(&line);
  if (!mTestFile)
    return PARSE_ERROR;

  // consume whitespace between args
  NS_tchar *q = mstrtok(kQuote, &line);
  if (!q)
    return PARSE_ERROR;

  return AddFile::Parse(line);
}

int
AddIfFile::Prepare()
{
  // If the test file does not exist, then skip this action.
  if (NS_taccess(mTestFile, F_OK)) {
    mTestFile = nullptr;
    return OK;
  }

  return AddFile::Prepare();
}

int
AddIfFile::Execute()
{
  if (!mTestFile)
    return OK;

  return AddFile::Execute();
}

void
AddIfFile::Finish(int status)
{
  if (!mTestFile)
    return;

  AddFile::Finish(status);
}

class AddIfNotFile : public AddFile
{
public:
  AddIfNotFile() : mTestFile(NULL) { }

  virtual int Parse(NS_tchar *line);
  virtual int Prepare();
  virtual int Execute();
  virtual void Finish(int status);

protected:
  const NS_tchar *mTestFile;
};

int
AddIfNotFile::Parse(NS_tchar *line)
{
  // format "<testfile>" "<newfile>"

  mTestFile = get_valid_path(&line);
  if (!mTestFile)
    return PARSE_ERROR;

  // consume whitespace between args
  NS_tchar *q = mstrtok(kQuote, &line);
  if (!q)
    return PARSE_ERROR;

  return AddFile::Parse(line);
}

int
AddIfNotFile::Prepare()
{
  // If the test file exists, then skip this action.
  if (!NS_taccess(mTestFile, F_OK)) {
    mTestFile = NULL;
    return OK;
  }

  return AddFile::Prepare();
}

int
AddIfNotFile::Execute()
{
  if (!mTestFile)
    return OK;

  return AddFile::Execute();
}

void
AddIfNotFile::Finish(int status)
{
  if (!mTestFile)
    return;

  AddFile::Finish(status);
}

class PatchIfFile : public PatchFile
{
public:
  PatchIfFile() : mTestFile(nullptr) { }

  virtual int Parse(NS_tchar *line);
  virtual int Prepare(); // should check for patch file and for checksum here
  virtual int Execute();
  virtual void Finish(int status);

private:
  const NS_tchar *mTestFile;
};

int
PatchIfFile::Parse(NS_tchar *line)
{
  // format "<testfile>" "<patchfile>" "<filetopatch>"

  mTestFile = get_valid_path(&line);
  if (!mTestFile)
    return PARSE_ERROR;

  // consume whitespace between args
  NS_tchar *q = mstrtok(kQuote, &line);
  if (!q)
    return PARSE_ERROR;

  return PatchFile::Parse(line);
}

int
PatchIfFile::Prepare()
{
  // If the test file does not exist, then skip this action.
  if (NS_taccess(mTestFile, F_OK)) {
    mTestFile = nullptr;
    return OK;
  }

  return PatchFile::Prepare();
}

int
PatchIfFile::Execute()
{
  if (!mTestFile)
    return OK;

  return PatchFile::Execute();
}

void
PatchIfFile::Finish(int status)
{
  if (!mTestFile)
    return;

  PatchFile::Finish(status);
}

//-----------------------------------------------------------------------------

#ifdef XP_WIN
#include "nsWindowsRestart.cpp"
#include "nsWindowsHelpers.h"
#include "uachelper.h"
#include "pathhash.h"

#ifdef MOZ_METRO
/**
 * Determines if the update came from an Immersive browser
 * @return true if the update came from an immersive browser
 */
bool
IsUpdateFromMetro(int argc, NS_tchar **argv)
{
  for (int i = 0; i < argc; i++) {
    if (!wcsicmp(L"-ServerName:DefaultBrowserServer", argv[i])) {
      return true;
    }
  }
  return false;
}
#endif

/**
 * Launch the post update application (helper.exe). It takes in the path of the
 * callback application to calculate the path of helper.exe. For service updates
 * this is called from both the system account and the current user account.
 *
 * @param  installationDir The path to the callback application binary.
 * @param  updateInfoDir   The directory where update info is stored.
 * @return true if there was no error starting the process.
 */
bool
LaunchWinPostProcess(const WCHAR *installationDir,
                     const WCHAR *updateInfoDir)
{
  WCHAR workingDirectory[MAX_PATH + 1] = { L'\0' };
  wcsncpy(workingDirectory, installationDir, MAX_PATH);

  // Launch helper.exe to perform post processing (e.g. registry and log file
  // modifications) for the update.
  WCHAR inifile[MAX_PATH + 1] = { L'\0' };
  wcsncpy(inifile, installationDir, MAX_PATH);
  if (!PathAppendSafe(inifile, L"updater.ini")) {
    return false;
  }

  WCHAR exefile[MAX_PATH + 1];
  WCHAR exearg[MAX_PATH + 1];
  WCHAR exeasync[10];
  bool async = true;
  if (!GetPrivateProfileStringW(L"PostUpdateWin", L"ExeRelPath", nullptr,
                                exefile, MAX_PATH + 1, inifile)) {
    return false;
  }

  if (!GetPrivateProfileStringW(L"PostUpdateWin", L"ExeArg", nullptr, exearg,
                                MAX_PATH + 1, inifile)) {
    return false;
  }

  if (!GetPrivateProfileStringW(L"PostUpdateWin", L"ExeAsync", L"TRUE",
                                exeasync,
                                sizeof(exeasync)/sizeof(exeasync[0]),
                                inifile)) {
    return false;
  }

  // Verify that exeFile doesn't contain relative paths
  if (wcsstr(exefile, L"..") != nullptr) {
    return false;
  }

  WCHAR exefullpath[MAX_PATH + 1] = { L'\0' };
  wcsncpy(exefullpath, installationDir, MAX_PATH);
  if (!PathAppendSafe(exefullpath, exefile)) {
    return false;
  }

// TEST_UPDATER is not available on esr38
//#if !defined(TEST_UPDATER)
  if (sUsingService &&
      !DoesBinaryMatchAllowedCertificates(installationDir, exefullpath)) {
    return false;
  }
//#endif

  WCHAR dlogFile[MAX_PATH + 1];
  if (!PathGetSiblingFilePath(dlogFile, exefullpath, L"uninstall.update")) {
    return false;
  }

  WCHAR slogFile[MAX_PATH + 1] = { L'\0' };
  wcsncpy(slogFile, updateInfoDir, MAX_PATH);
  if (!PathAppendSafe(slogFile, L"update.log")) {
    return false;
  }

  WCHAR dummyArg[14] = { L'\0' };
  wcsncpy(dummyArg, L"argv0ignored ", sizeof(dummyArg) / sizeof(dummyArg[0]) - 1);

  size_t len = wcslen(exearg) + wcslen(dummyArg);
  WCHAR *cmdline = (WCHAR *) malloc((len + 1) * sizeof(WCHAR));
  if (!cmdline) {
    return false;
  }

  wcsncpy(cmdline, dummyArg, len);
  wcscat(cmdline, exearg);

  if (sUsingService ||
      !_wcsnicmp(exeasync, L"false", 6) ||
      !_wcsnicmp(exeasync, L"0", 2)) {
    async = false;
  }

  // We want to launch the post update helper app to update the Windows
  // registry even if there is a failure with removing the uninstall.update
  // file or copying the update.log file.
  CopyFileW(slogFile, dlogFile, false);

  STARTUPINFOW si = {sizeof(si), 0};
  si.lpDesktop = L"";
  PROCESS_INFORMATION pi = {0};

  bool ok = CreateProcessW(exefullpath,
                           cmdline,
                           nullptr,  // no special security attributes
                           nullptr,  // no special thread attributes
                           false,    // don't inherit filehandles
                           0,        // No special process creation flags
                           nullptr,  // inherit my environment
                           workingDirectory,
                           &si,
                           &pi);
  free(cmdline);
  if (ok) {
    if (!async) {
      WaitForSingleObject(pi.hProcess, INFINITE);
    }
    CloseHandle(pi.hProcess);
    CloseHandle(pi.hThread);
  }
  return ok;
}

#endif

static void
LaunchCallbackApp(const NS_tchar *workingDir,
                  int argc,
                  NS_tchar **argv,
                  bool usingService)
{
  putenv(const_cast<char*>("NO_EM_RESTART="));
  putenv(const_cast<char*>("MOZ_LAUNCHED_CHILD=1"));

  // Run from the specified working directory (see bug 312360). This is not
  // necessary on Windows CE since the application that launches the updater
  // passes the working directory as an --environ: command line argument.
  if (NS_tchdir(workingDir) != 0) {
    LOG(("Warning: chdir failed"));
  }

#if defined(USE_EXECV)
  execv(argv[0], argv);
#elif defined(XP_MACOSX)
  LaunchChild(argc, argv);
#elif defined(XP_WIN)
  // Do not allow the callback to run when running an update through the
  // service as session 0.  The unelevated updater.exe will do the launching.
  if (!usingService) {
#if defined(MOZ_METRO)
    // If our callback application is the default metro browser, then
    // launch it now.
    if (IsUpdateFromMetro(argc, argv)) {
      LaunchDefaultMetroBrowser();
      return;
    }
#endif
    WinLaunchChild(argv[0], argc, argv, nullptr);
  }
#else
# warning "Need implementaton of LaunchCallbackApp"
#endif
}

static bool
WriteStatusFile(const char* aStatus)
{
  NS_tchar filename[MAXPATHLEN] = {NS_T('\0')};
#if defined(XP_WIN)
  // The temp file is not removed on failure since there is client code that
  // will remove it.
  GetTempFileNameW(gPatchDirPath, L"sta", 0, filename);
#else
  NS_tsnprintf(filename, sizeof(filename)/sizeof(filename[0]),
               NS_T("%s/update.status"), gPatchDirPath);
#endif

  // Make sure that the directory for the update status file exists
  if (ensure_parent_dir(filename)) {
    return false;
  }

  // This is scoped to make the AutoFile close the file so it is possible to
  // move the temp file to the update.status file on Windows.
  {
    AutoFile file(NS_tfopen(filename, NS_T("wb+")));
    if (file == nullptr) {
      return false;
    }

    if (fwrite(aStatus, strlen(aStatus), 1, file) != 1) {
      return false;
    }
  }

#if defined(XP_WIN)
  NS_tchar dstfilename[MAXPATHLEN] = {NS_T('\0')};
  NS_tsnprintf(dstfilename, sizeof(dstfilename)/sizeof(dstfilename[0]),
               NS_T("%s\\update.status"), gPatchDirPath);
  if (MoveFileExW(filename, dstfilename, MOVEFILE_REPLACE_EXISTING) == 0) {
    return false;
  }
#endif

  return true;
}

static void
WriteStatusFile(int status)
{
  const char *text;

  char buf[32];
  if (status == OK) {
    if (sStagedUpdate) {
      text = "applied\n";
    } else {
      text = "succeeded\n";
    }
  } else {
    snprintf(buf, sizeof(buf)/sizeof(buf[0]), "failed: %d\n", status);
    text = buf;
  }

  WriteStatusFile(text);
}

#ifdef MOZ_MAINTENANCE_SERVICE
/*
 * Read the update.status file and sets isPendingService to true if
 * the status is set to pending-service.
 *
 * @param  isPendingService Out parameter for specifying if the status
 *         is set to pending-service or not.
 * @return true if the information was retrieved and it is pending
 *         or pending-service.
*/
static bool
IsUpdateStatusPendingService()
{
  NS_tchar filename[MAXPATHLEN];
  NS_tsnprintf(filename, sizeof(filename)/sizeof(filename[0]),
               NS_T("%s/update.status"), gPatchDirPath);

  AutoFile file(NS_tfopen(filename, NS_T("rb")));
  if (file == nullptr)
    return false;

  char buf[32] = { 0 };
  fread(buf, sizeof(buf), 1, file);

  const char kPendingService[] = "pending-service";
  const char kAppliedService[] = "applied-service";

  return (strncmp(buf, kPendingService,
                  sizeof(kPendingService) - 1) == 0) ||
         (strncmp(buf, kAppliedService,
                  sizeof(kAppliedService) - 1) == 0);
}
#endif

#ifdef XP_WIN
/*
 * Read the update.status file and sets isSuccess to true if
 * the status is set to succeeded.
 *
 * @param  isSucceeded Out parameter for specifying if the status
 *         is set to succeeded or not.
 * @return true if the information was retrieved and it is succeeded.
*/
static bool
IsUpdateStatusSucceeded(bool &isSucceeded)
{
  isSucceeded = false;
  NS_tchar filename[MAXPATHLEN];
  NS_tsnprintf(filename, sizeof(filename)/sizeof(filename[0]),
               NS_T("%s/update.status"), gPatchDirPath);

  AutoFile file(NS_tfopen(filename, NS_T("rb")));
  if (file == nullptr)
    return false;

  char buf[32] = { 0 };
  fread(buf, sizeof(buf), 1, file);

  const char kSucceeded[] = "succeeded";
  isSucceeded = strncmp(buf, kSucceeded,
                        sizeof(kSucceeded) - 1) == 0;
  return true;
}
#endif

/*
 * Copy the entire contents of the application installation directory to the
 * destination directory for the update process.
 *
 * @return 0 if successful, an error code otherwise.
 */
static int
CopyInstallDirToDestDir()
{
  // These files should not be copied over to the updated app
#ifdef XP_WIN
#define SKIPLIST_COUNT 3
#elif XP_MACOSX
#define SKIPLIST_COUNT 0
#else
#define SKIPLIST_COUNT 2
#endif
  copy_recursive_skiplist<SKIPLIST_COUNT> skiplist;
#ifndef XP_MACOSX
  skiplist.append(0, gInstallDirPath, NS_T("updated"));
  skiplist.append(1, gInstallDirPath, NS_T("updates/0"));
#ifdef XP_WIN
  skiplist.append(2, gInstallDirPath, NS_T("updated.update_in_progress.lock"));
#endif
#endif

  return ensure_copy_recursive(gInstallDirPath, gWorkingDirPath, skiplist);
}

/*
 * Replace the application installation directory with the destination
 * directory in order to finish a staged update task
 *
 * @return 0 if successful, an error code otherwise.
 */
static int
ProcessReplaceRequest()
{
  // The replacement algorithm is like this:
  // 1. Move destDir to tmpDir.  In case of failure, abort.
  // 2. Move newDir to destDir.  In case of failure, revert step 1 and abort.
  // 3. Delete tmpDir (or defer it to the next reboot).

#ifdef XP_MACOSX
  NS_tchar destDir[MAXPATHLEN];
  NS_tsnprintf(destDir, sizeof(destDir)/sizeof(destDir[0]),
               NS_T("%s/Contents"), gInstallDirPath);
#elif XP_WIN
  // Windows preserves the case of the file/directory names.  We use the
  // GetLongPathName API in order to get the correct case for the directory
  // name, so that if the user has used a different case when launching the
  // application, the installation directory's name does not change.
  NS_tchar destDir[MAXPATHLEN];
  if (!GetLongPathNameW(gInstallDirPath, destDir, sizeof(destDir)/sizeof(destDir[0]))) {
    return NO_INSTALLDIR_ERROR;
  }
#else
  NS_tchar* destDir = gInstallDirPath;
#endif

  NS_tchar tmpDir[MAXPATHLEN];
  NS_tsnprintf(tmpDir, sizeof(tmpDir)/sizeof(tmpDir[0]),
               NS_T("%s.bak"), destDir);

  NS_tchar newDir[MAXPATHLEN];
  NS_tsnprintf(newDir, sizeof(newDir)/sizeof(newDir[0]),
#ifdef XP_MACOSX
               NS_T("%s/Contents"),
               gWorkingDirPath);
#else
               NS_T("%s.bak/updated"),
               gInstallDirPath);
#endif

  // First try to remove the possibly existing temp directory, because if this
  // directory exists, we will fail to rename destDir.
  // No need to error check here because if this fails, we will fail in the
  // next step anyways.
  ensure_remove_recursive(tmpDir);

  LOG(("Begin moving destDir (" LOG_S ") to tmpDir (" LOG_S ")",
       destDir, tmpDir));
  int rv = rename_file(destDir, tmpDir, true);
#ifdef XP_WIN
  // On Windows, if Firefox is launched using the shortcut, it will hold a handle
  // to its installation directory open, which might not get released in time.
  // Therefore we wait a little bit here to see if the handle is released.
  // If it's not released, we just fail to perform the replace request.
  const int max_retries = 10;
  int retries = 0;
  while (rv == WRITE_ERROR && (retries++ < max_retries)) {
    LOG(("PerformReplaceRequest: destDir rename attempt %d failed. " \
         "File: " LOG_S ". Last error: %d, err: %d", retries,
         destDir, GetLastError(), rv));

    Sleep(100);

    rv = rename_file(destDir, tmpDir, true);
  }
#endif
  if (rv) {
    LOG(("Moving destDir to tmpDir failed, err: %d", rv));
    return rv;
  }

  LOG(("Begin moving newDir (" LOG_S ") to destDir (" LOG_S ")",
       newDir, destDir));
  rv = rename_file(newDir, destDir, true);
#ifdef XP_MACOSX
  if (rv) {
    LOG(("Moving failed. Begin copying newDir (" LOG_S ") to destDir (" LOG_S ")",
         newDir, destDir));
    copy_recursive_skiplist<0> skiplist;
    rv = ensure_copy_recursive(newDir, destDir, skiplist);
  }
#endif
  if (rv) {
    LOG(("Moving newDir to destDir failed, err: %d", rv));
    LOG(("Now, try to move tmpDir back to destDir"));
    ensure_remove_recursive(destDir);
    int rv2 = rename_file(tmpDir, destDir, true);
    if (rv2) {
      LOG(("Moving tmpDir back to destDir failed, err: %d", rv2));
    }
    return rv;
  }

  LOG(("Now, remove the tmpDir"));
  rv = ensure_remove_recursive(tmpDir, true);
  if (rv) {
    LOG(("Removing tmpDir failed, err: %d", rv));
#ifdef XP_WIN
    NS_tchar deleteDir[MAXPATHLEN];
    NS_tsnprintf(deleteDir, sizeof(deleteDir)/sizeof(deleteDir[0]),
                 NS_T("%s\\%s"), destDir, DELETE_DIR);
    // Attempt to remove the tobedeleted directory and then recreate it if it
    // was successfully removed.
    _wrmdir(deleteDir);
    if (NS_taccess(deleteDir, F_OK)) {
      NS_tmkdir(deleteDir, 0755);
    }
    remove_recursive_on_reboot(tmpDir, deleteDir);
#endif
  }

#ifdef XP_MACOSX
  // On OS X, we we need to remove the staging directory after its Contents
  // directory has been moved.
  NS_tchar updatedAppDir[MAXPATHLEN];
  NS_tsnprintf(updatedAppDir, sizeof(updatedAppDir)/sizeof(updatedAppDir[0]),
               NS_T("%s/Updated.app"), gPatchDirPath);
  ensure_remove_recursive(updatedAppDir);
#endif

  gSucceeded = true;

  return 0;
}

#ifdef XP_WIN
static void
WaitForServiceFinishThread(void *param)
{
  // We wait at most 10 minutes, we already waited 5 seconds previously
  // before deciding to show this UI.
  WaitForServiceStop(SVC_NAME, 595);
  QuitProgressUI();
}
#endif

#ifdef MOZ_VERIFY_MAR_SIGNATURE
/**
 * This function reads in the ACCEPTED_MAR_CHANNEL_IDS from update-settings.ini
 *
 * @param path    The path to the ini file that is to be read
 * @param results A pointer to the location to store the read strings
 * @return OK on success
 */
static int
ReadMARChannelIDs(const NS_tchar *path, MARChannelStringTable *results)
{
  const unsigned int kNumStrings = 1;
  const char *kUpdaterKeys = "ACCEPTED_MAR_CHANNEL_IDS\0";
  char updater_strings[kNumStrings][MAX_TEXT_LEN];

  int result = ReadStrings(path, kUpdaterKeys, kNumStrings,
                           updater_strings, "Settings");

  strncpy(results->MARChannelID, updater_strings[0], MAX_TEXT_LEN - 1);
  results->MARChannelID[MAX_TEXT_LEN - 1] = 0;

  return result;
}
#endif

static int
GetUpdateFileName(NS_tchar *fileName, int maxChars)
{
#if defined(MOZ_WIDGET_GONK)  // If an update.link file exists, then it will contain the name
  // of the update file (terminated by a newline).

  NS_tchar linkFileName[MAXPATHLEN];
  NS_tsnprintf(linkFileName, sizeof(linkFileName)/sizeof(linkFileName[0]),
               NS_T("%s/update.link"), gPatchDirPath);
  AutoFile linkFile(NS_tfopen(linkFileName, NS_T("rb")));
  if (linkFile == nullptr) {
    NS_tsnprintf(fileName, maxChars,
                 NS_T("%s/update.mar"), gPatchDirPath);
    return OK;
  }

  char dataFileName[MAXPATHLEN];
  size_t bytesRead;

  if ((bytesRead = fread(dataFileName, 1, sizeof(dataFileName)-1, linkFile)) <= 0) {
    *fileName = NS_T('\0');
    return READ_ERROR;
  }
  if (dataFileName[bytesRead-1] == '\n') {
    // Strip trailing newline (for \n and \r\n)
    bytesRead--;
  }
  if (dataFileName[bytesRead-1] == '\r') {
    // Strip trailing CR (for \r, \r\n)
    bytesRead--;
  }
  dataFileName[bytesRead] = '\0';

  strncpy(fileName, dataFileName, maxChars-1);
  fileName[maxChars-1] = '\0';
#else
  // We currently only support update.link files under GONK
  NS_tsnprintf(fileName, maxChars,
               NS_T("%s/update.mar"), gPatchDirPath);
#endif
  return OK;
}

static void
UpdateThreadFunc(void *param)
{
  // open ZIP archive and process...
  int rv;
  if (sReplaceRequest) {
    rv = ProcessReplaceRequest();
  } else {
    NS_tchar dataFile[MAXPATHLEN];
    rv = GetUpdateFileName(dataFile, sizeof(dataFile)/sizeof(dataFile[0]));
    if (rv == OK) {
      rv = gArchiveReader.Open(dataFile);
    }

#ifdef MOZ_VERIFY_MAR_SIGNATURE
    if (rv == OK) {
#ifdef XP_WIN
      HKEY baseKey = nullptr;
      wchar_t valueName[] = L"Image Path";
      wchar_t rasenh[] = L"rsaenh.dll";
      bool reset = false;
      if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                        L"SOFTWARE\\Microsoft\\Cryptography\\Defaults\\Provider\\Microsoft Enhanced Cryptographic Provider v1.0",
                        0, KEY_READ | KEY_WRITE,
                        &baseKey) == ERROR_SUCCESS) {
        wchar_t path[MAX_PATH + 1];
        DWORD size = sizeof(path);
        DWORD type;
        if (RegQueryValueExW(baseKey, valueName, 0, &type,
                             (LPBYTE)path, &size) == ERROR_SUCCESS) {
          if (type == REG_SZ && wcscmp(path, rasenh) == 0) {
            wchar_t rasenhFullPath[] = L"%SystemRoot%\\System32\\rsaenh.dll";
            if (RegSetValueExW(baseKey, valueName, 0, REG_SZ,
                               (const BYTE*)rasenhFullPath,
                               sizeof(rasenhFullPath)) == ERROR_SUCCESS) {
              reset = true;
            }
          }
        }
      }
#endif
      rv = gArchiveReader.VerifySignature();
#ifdef XP_WIN
      if (baseKey) {
        if (reset) {
          RegSetValueExW(baseKey, valueName, 0, REG_SZ,
                         (const BYTE*)rasenh,
                         sizeof(rasenh));
        }
        RegCloseKey(baseKey);
      }
#endif
    }

    if (rv == OK) {
      if (rv == OK) {
        NS_tchar updateSettingsPath[MAX_TEXT_LEN];
        NS_tsnprintf(updateSettingsPath,
                     sizeof(updateSettingsPath) / sizeof(updateSettingsPath[0]),
                     NS_T("%s/update-settings.ini"), gWorkingDirPath);
        MARChannelStringTable MARStrings;
        if (ReadMARChannelIDs(updateSettingsPath, &MARStrings) != OK) {
          // If we can't read from update-settings.ini then we shouldn't impose
          // a MAR restriction.  Some installations won't even include this file.
          MARStrings.MARChannelID[0] = '\0';
        }

        rv = gArchiveReader.VerifyProductInformation(MARStrings.MARChannelID,
                                                     MOZ_APP_VERSION);
      }
    }
#endif

    if (rv == OK && sStagedUpdate && !sIsOSUpdate) {
      rv = CopyInstallDirToDestDir();
    }

    if (rv == OK) {
      rv = DoUpdate();
      gArchiveReader.Close();
      NS_tchar updatingDir[MAXPATHLEN];
      NS_tsnprintf(updatingDir, sizeof(updatingDir)/sizeof(updatingDir[0]),
                   NS_T("%s/updating"), gWorkingDirPath);
      ensure_remove_recursive(updatingDir);
    }
  }

  bool reportRealResults = true;
  if (sReplaceRequest && rv && !getenv("MOZ_NO_REPLACE_FALLBACK")) {
    // When attempting to replace the application, we should fall back
    // to non-staged updates in case of a failure.  We do this by
    // setting the status to pending, exiting the updater, and
    // launching the callback application.  The callback application's
    // startup path will see the pending status, and will start the
    // updater application again in order to apply the update without
    // staging.
    // The MOZ_NO_REPLACE_FALLBACK environment variable is used to
    // bypass this fallback, and is used in the updater tests.
    // The only special thing which we should do here is to remove the
    // staged directory as it won't be useful any more.
    ensure_remove_recursive(gWorkingDirPath);
    WriteStatusFile(sUsingService ? "pending-service" : "pending");
    putenv(const_cast<char*>("MOZ_PROCESS_UPDATES=")); // We need to use --process-updates again in the tests
    reportRealResults = false; // pretend success
  }

  if (reportRealResults) {
    if (rv) {
      LOG(("failed: %d", rv));
    }
    else {
#ifdef XP_MACOSX
      // If the update was successful we need to update the timestamp on the
      // top-level Mac OS X bundle directory so that Mac OS X's Launch Services
      // picks up any major changes when the bundle is updated.
      if (!sStagedUpdate && utimes(gInstallDirPath, nullptr) != 0) {
        LOG(("Couldn't set access/modification time on application bundle."));
      }
#endif

      LOG(("succeeded"));
    }
    WriteStatusFile(rv);
  }

  LOG(("calling QuitProgressUI"));
  QuitProgressUI();
}

int NS_main(int argc, NS_tchar **argv)
{
#if defined(MOZ_WIDGET_GONK)
  if (getenv("LD_PRELOAD")) {
    // If the updater is launched with LD_PRELOAD set, then we wind up
    // preloading libmozglue.so. Under some circumstances, this can cause
    // the remount of /system to fail when going from rw to ro, so if we
    // detect LD_PRELOAD we unsetenv it and relaunch ourselves without it.
    // This will cause the offending preloaded library to be closed.
    //
    // For a variety of reasons, this is really hard to do in a safe manner
    // in the parent process, so we do it here.
    unsetenv("LD_PRELOAD");
    execv(argv[0], argv);
    __android_log_print(ANDROID_LOG_INFO, "updater",
                        "execve failed: errno: %d. Exiting...", errno);
    _exit(1);
  }
#endif
  InitProgressUI(&argc, &argv);

  // To process an update the updater command line must at a minimum have the
  // directory path containing the updater.mar file to process as the first
  // argument, the install directory as the second argument, and the directory
  // to apply the update to as the third argument. When the updater is launched
  // by another process the PID of the parent process should be provided in the
  // optional fourth argument and the updater will wait on the parent process to
  // exit if the value is non-zero and the process is present. This is necessary
  // due to not being able to update files that are in use on Windows. The
  // optional fifth argument is the callback's working directory and the
  // optional sixth argument is the callback path. The callback is the
  // application to launch after updating and it will be launched when these
  // arguments are provided whether the update was successful or not. All
  // remaining arguments are optional and are passed to the callback when it is
  // launched.
  if (argc < 4) {
    fprintf(stderr, "Usage: updater patch-dir install-dir apply-to-dir [wait-pid [callback-working-dir callback-path args...]]\n");
    return 1;
  }

  // The directory containing the update information.
  gPatchDirPath = argv[1];
  // The directory we're going to update to.
  // We copy this string because we need to remove trailing slashes.  The C++
  // standard says that it's always safe to write to strings pointed to by argv
  // elements, but I don't necessarily believe it.
  NS_tstrncpy(gInstallDirPath, argv[2], MAXPATHLEN);
  gInstallDirPath[MAXPATHLEN - 1] = NS_T('\0');
  NS_tchar *slash = NS_tstrrchr(gInstallDirPath, NS_SLASH);
  if (slash && !slash[1]) {
    *slash = NS_T('\0');
  }

#ifdef XP_WIN
  bool useService = false;
  bool testOnlyFallbackKeyExists = false;
  bool noServiceFallback = getenv("MOZ_NO_SERVICE_FALLBACK") != nullptr;
  putenv(const_cast<char*>("MOZ_NO_SERVICE_FALLBACK="));

  // We never want the service to be used unless we build with
  // the maintenance service.
#ifdef MOZ_MAINTENANCE_SERVICE
  useService = IsUpdateStatusPendingService();
  // Our tests run with a different apply directory for each test.
  // We use this registry key on our test slaves to store the
  // allowed name/issuers.
  testOnlyFallbackKeyExists = DoesFallbackKeyExist();
#endif

  // Remove everything except close window from the context menu
  {
    HKEY hkApp = nullptr;
    RegCreateKeyExW(HKEY_CURRENT_USER, L"Software\\Classes\\Applications",
                    0, nullptr, REG_OPTION_NON_VOLATILE, KEY_SET_VALUE, nullptr,
                    &hkApp, nullptr);
    RegCloseKey(hkApp);
    if (RegCreateKeyExW(HKEY_CURRENT_USER,
                        L"Software\\Classes\\Applications\\updater.exe",
                        0, nullptr, REG_OPTION_VOLATILE, KEY_SET_VALUE, nullptr,
                        &hkApp, nullptr) == ERROR_SUCCESS) {
      RegSetValueExW(hkApp, L"IsHostApp", 0, REG_NONE, 0, 0);
      RegSetValueExW(hkApp, L"NoOpenWith", 0, REG_NONE, 0, 0);
      RegSetValueExW(hkApp, L"NoStartPage", 0, REG_NONE, 0, 0);
      RegCloseKey(hkApp);
    }
  }
#endif

  // If there is a PID specified and it is not '0' then wait for the process to exit.
#ifdef XP_WIN
  __int64 pid = 0;
#else
  int pid = 0;
#endif
  if (argc > 4) {
#ifdef XP_WIN
    pid = _wtoi64(argv[4]);
#else
    pid = atoi(argv[4]);
#endif
    if (pid == -1) {
      // This is a signal from the parent process that the updater should stage
      // the update.
      sStagedUpdate = true;
    } else if (NS_tstrstr(argv[4], NS_T("/replace"))) {
      // We're processing a request to replace the application with a staged
      // update.
      sReplaceRequest = true;
    }
  }

  // The directory we're going to update to.
  // We copy this string because we need to remove trailing slashes.  The C++
  // standard says that it's always safe to write to strings pointed to by argv
  // elements, but I don't necessarily believe it.
  NS_tstrncpy(gWorkingDirPath, argv[3], MAXPATHLEN);
  gWorkingDirPath[MAXPATHLEN - 1] = NS_T('\0');
  slash = NS_tstrrchr(gWorkingDirPath, NS_SLASH);
  if (slash && !slash[1]) {
    *slash = NS_T('\0');
  }

  if (getenv("MOZ_OS_UPDATE")) {
    sIsOSUpdate = true;
    putenv(const_cast<char*>("MOZ_OS_UPDATE="));
  }

  if (sReplaceRequest) {
    // If we're attempting to replace the application, try to append to the
    // log generated when staging the staged update.
#ifdef XP_WIN
    NS_tchar* logDir = gPatchDirPath;
#else
#ifdef XP_MACOSX
    NS_tchar* logDir = gPatchDirPath;
#else
    NS_tchar logDir[MAXPATHLEN];
    NS_tsnprintf(logDir, sizeof(logDir)/sizeof(logDir[0]),
                 NS_T("%s/updated/updates"),
                 gInstallDirPath);
#endif
#endif

    LogInitAppend(logDir, NS_T("last-update.log"), NS_T("update.log"));
  } else {
    LogInit(gPatchDirPath, NS_T("update.log"));
  }

  if (!WriteStatusFile("applying")) {
    LOG(("failed setting status to 'applying'"));
    return 1;
  }

  if (sStagedUpdate) {
    LOG(("Performing a staged update"));
  } else if (sReplaceRequest) {
    LOG(("Performing a replace request"));
  }

  LOG(("PATCH DIRECTORY " LOG_S, gPatchDirPath));
  LOG(("INSTALLATION DIRECTORY " LOG_S, gInstallDirPath));
  LOG(("WORKING DIRECTORY " LOG_S, gWorkingDirPath));

#if defined(XP_WIN)
  if (sReplaceRequest || sStagedUpdate) {
    NS_tchar stagedParent[MAX_PATH];
    NS_tsnprintf(stagedParent, sizeof(stagedParent)/sizeof(stagedParent[0]),
                 NS_T("%s"), gWorkingDirPath);
    if (!PathRemoveFileSpecW(stagedParent)) {
      WriteStatusFile(REMOVE_FILE_SPEC_ERROR);
      LOG(("Error calling PathRemoveFileSpecW: %d", GetLastError()));
      LogFinish();
      return 1;
    }

    if (_wcsnicmp(stagedParent, gInstallDirPath, MAX_PATH) != 0) {
      WriteStatusFile(INVALID_STAGED_PARENT_ERROR);
      LOG(("Stage and Replace requests require that the working directory " \
           "is a sub-directory of the installation directory! Exiting"));
      LogFinish();
      return 1;
    }
  }
#endif

#ifdef MOZ_WIDGET_GONK
  const char *prioEnv = getenv("MOZ_UPDATER_PRIO");
  if (prioEnv) {
    int32_t prioVal;
    int32_t oomScoreAdj;
    int32_t ioprioClass;
    int32_t ioprioLevel;
    if (sscanf(prioEnv, "%d/%d/%d/%d",
               &prioVal, &oomScoreAdj, &ioprioClass, &ioprioLevel) == 4) {
      LOG(("MOZ_UPDATER_PRIO=%s", prioEnv));
      if (setpriority(PRIO_PROCESS, 0, prioVal)) {
        LOG(("setpriority(%d) failed, errno = %d", prioVal, errno));
      }
      if (ioprio_set(IOPRIO_WHO_PROCESS, 0,
                     IOPRIO_PRIO_VALUE(ioprioClass, ioprioLevel))) {
        LOG(("ioprio_set(%d,%d) failed: errno = %d",
             ioprioClass, ioprioLevel, errno));
      }
      FILE *fs = fopen("/proc/self/oom_score_adj", "w");
      if (fs) {
        fprintf(fs, "%d", oomScoreAdj);
        fclose(fs);
      } else {
        LOG(("Unable to open /proc/self/oom_score_adj for writing, errno = %d", errno));
      }
    }
  }
#endif

#ifdef XP_WIN
  if (pid > 0) {
    HANDLE parent = OpenProcess(SYNCHRONIZE, false, (DWORD) pid);
    // May return nullptr if the parent process has already gone away.
    // Otherwise, wait for the parent process to exit before starting the
    // update.
    if (parent) {
      bool updateFromMetro = false;
#ifdef MOZ_METRO
      updateFromMetro = IsUpdateFromMetro(argc, argv);
#endif
      DWORD waitTime = updateFromMetro ?
                       IMMERSIVE_PARENT_WAIT : PARENT_WAIT;
      DWORD result = WaitForSingleObject(parent, waitTime);
      CloseHandle(parent);
      if (result != WAIT_OBJECT_0 && !updateFromMetro)
        return 1;
    }
  }
#else
  if (pid > 0)
    waitpid(pid, nullptr, 0);
#endif

  if (sReplaceRequest) {
#ifdef XP_WIN
    // On Windows, the current working directory of the process should be changed
    // so that it's not locked.
    NS_tchar sysDir[MAX_PATH + 1] = { L'\0' };
    if (GetSystemDirectoryW(sysDir, MAX_PATH + 1)) {
      NS_tchdir(sysDir);
    }
#endif
  }

  // The callback is the remaining arguments starting at callbackIndex.
  // The argument specified by callbackIndex is the callback executable and the
  // argument prior to callbackIndex is the working directory.
  const int callbackIndex = 6;

#if defined(XP_WIN)
  sUsingService = getenv("MOZ_USING_SERVICE") != nullptr;
  putenv(const_cast<char*>("MOZ_USING_SERVICE="));
  // lastFallbackError keeps track of the last error for the service not being
  // used, in case of an error when fallback is not enabled we write the
  // error to the update.status file.
  // When fallback is disabled (MOZ_NO_SERVICE_FALLBACK does not exist) then
  // we will instead fallback to not using the service and display a UAC prompt.
  int lastFallbackError = FALLBACKKEY_UNKNOWN_ERROR;

  // Launch a second instance of the updater with the runas verb on Windows
  // when write access is denied to the installation directory.
  HANDLE updateLockFileHandle = INVALID_HANDLE_VALUE;
  NS_tchar elevatedLockFilePath[MAXPATHLEN] = {NS_T('\0')};
  if (!sUsingService &&
      (argc > callbackIndex || sStagedUpdate || sReplaceRequest)) {
    NS_tchar updateLockFilePath[MAXPATHLEN];
    if (sStagedUpdate) {
      // When staging an update, the lock file is:
      // <install_dir>\updated.update_in_progress.lock
      NS_tsnprintf(updateLockFilePath,
                   sizeof(updateLockFilePath)/sizeof(updateLockFilePath[0]),
                   NS_T("%s/updated.update_in_progress.lock"), gInstallDirPath);
    } else if (sReplaceRequest) {
      // When processing a replace request, the lock file is:
      // <install_dir>\..\moz_update_in_progress.lock
      NS_tchar installDir[MAXPATHLEN];
      NS_tstrcpy(installDir, gInstallDirPath);
      NS_tchar *slash = (NS_tchar *) NS_tstrrchr(installDir, NS_SLASH);
      *slash = NS_T('\0');
      NS_tsnprintf(updateLockFilePath,
                   sizeof(updateLockFilePath)/sizeof(updateLockFilePath[0]),
                   NS_T("%s\\moz_update_in_progress.lock"), installDir);
    } else {
      // In the non-staging update case, the lock file is:
      // <install_dir>\<app_name>.exe.update_in_progress.lock
      NS_tsnprintf(updateLockFilePath,
                   sizeof(updateLockFilePath)/sizeof(updateLockFilePath[0]),
                   NS_T("%s.update_in_progress.lock"), argv[callbackIndex]);
    }

    // The update_in_progress.lock file should only exist during an update. In
    // case it exists attempt to remove it and exit if that fails to prevent
    // simultaneous updates occurring.
    if (!_waccess(updateLockFilePath, F_OK) &&
        NS_tremove(updateLockFilePath) != 0) {
      // Try to fall back to the old way of doing updates if a staged
      // update fails.
      if (sStagedUpdate || sReplaceRequest) {
        // Note that this could fail, but if it does, there isn't too much we
        // can do in order to recover anyways.
        WriteStatusFile("pending");
      }
      LOG(("Update already in progress! Exiting"));
      return 1;
    }

    updateLockFileHandle = CreateFileW(updateLockFilePath,
                                       GENERIC_READ | GENERIC_WRITE,
                                       0,
                                       nullptr,
                                       OPEN_ALWAYS,
                                       FILE_FLAG_DELETE_ON_CLOSE,
                                       nullptr);

    NS_tsnprintf(elevatedLockFilePath,
                 sizeof(elevatedLockFilePath)/sizeof(elevatedLockFilePath[0]),
                 NS_T("%s/update_elevated.lock"), gPatchDirPath);

    // Even if a file has no sharing access, you can still get its attributes
    bool startedFromUnelevatedUpdater =
      GetFileAttributesW(elevatedLockFilePath) != INVALID_FILE_ATTRIBUTES;

    // If we're running from the service, then we were started with the same
    // token as the service so the permissions are already dropped.  If we're
    // running from an elevated updater that was started from an unelevated
    // updater, then we drop the permissions here. We do not drop the
    // permissions on the originally called updater because we use its token
    // to start the callback application.
    if (startedFromUnelevatedUpdater) {
      // Disable every privilege we don't need. Processes started using
      // CreateProcess will use the same token as this process.
      UACHelper::DisablePrivileges(nullptr);
    }

    if (updateLockFileHandle == INVALID_HANDLE_VALUE ||
        (useService && testOnlyFallbackKeyExists && noServiceFallback)) {
      if (!_waccess(elevatedLockFilePath, F_OK) &&
          NS_tremove(elevatedLockFilePath) != 0) {
        fprintf(stderr, "Unable to create elevated lock file! Exiting\n");
        return 1;
      }

      HANDLE elevatedFileHandle;
      elevatedFileHandle = CreateFileW(elevatedLockFilePath,
                                       GENERIC_READ | GENERIC_WRITE,
                                       0,
                                       nullptr,
                                       OPEN_ALWAYS,
                                       FILE_FLAG_DELETE_ON_CLOSE,
                                       nullptr);

      if (elevatedFileHandle == INVALID_HANDLE_VALUE) {
        LOG(("Unable to create elevated lock file! Exiting"));
        return 1;
      }

      wchar_t *cmdLine = MakeCommandLine(argc - 1, argv + 1);
      if (!cmdLine) {
        CloseHandle(elevatedFileHandle);
        return 1;
      }

      // Make sure the path to the updater to use for the update is on local.
      // We do this check to make sure that file locking is available for
      // race condition security checks.
      if (useService) {
        BOOL isLocal = FALSE;
        useService = IsLocalFile(argv[0], isLocal) && isLocal;
      }

      // If we have unprompted elevation we should NOT use the service
      // for the update. Service updates happen with the SYSTEM account
      // which has more privs than we need to update with.
      // Windows 8 provides a user interface so users can configure this
      // behavior and it can be configured in the registry in all Windows
      // versions that support UAC.
      if (useService) {
        BOOL unpromptedElevation;
        if (IsUnpromptedElevation(unpromptedElevation)) {
          useService = !unpromptedElevation;
        }
      }

      // Make sure the service registry entries for the instsallation path
      // are available.  If not don't use the service.
      if (useService) {
        WCHAR maintenanceServiceKey[MAX_PATH + 1];
        if (CalculateRegistryPathFromFilePath(gInstallDirPath, maintenanceServiceKey)) {
          HKEY baseKey = nullptr;
          if (RegOpenKeyExW(HKEY_LOCAL_MACHINE,
                            maintenanceServiceKey, 0,
                            KEY_READ | KEY_WOW64_64KEY,
                            &baseKey) == ERROR_SUCCESS) {
            RegCloseKey(baseKey);
          } else {
            useService = testOnlyFallbackKeyExists;
            if (!useService) {
              lastFallbackError = FALLBACKKEY_NOKEY_ERROR;
            }
          }
        } else {
          useService = false;
          lastFallbackError = FALLBACKKEY_REGPATH_ERROR;
        }
      }

      // Originally we used to write "pending" to update.status before
      // launching the service command.  This is no longer needed now
      // since the service command is launched from updater.exe.  If anything
      // fails in between, we can fall back to using the normal update process
      // on our own.

      // If we still want to use the service try to launch the service
      // comamnd for the update.
      if (useService) {
        // If the update couldn't be started, then set useService to false so
        // we do the update the old way.
        DWORD ret = LaunchServiceSoftwareUpdateCommand(argc, (LPCWSTR *)argv);
        useService = (ret == ERROR_SUCCESS);
        // If the command was launched then wait for the service to be done.
        if (useService) {
          bool showProgressUI = false;
          // Never show the progress UI when staging updates.
          if (!sStagedUpdate) {
            // We need to call this separately instead of allowing ShowProgressUI
            // to initialize the strings because the service will move the
            // ini file out of the way when running updater.
            showProgressUI = !InitProgressUIStrings();
          }

          // Wait for the service to stop for 5 seconds.  If the service
          // has still not stopped then show an indeterminate progress bar.
          DWORD lastState = WaitForServiceStop(SVC_NAME, 5);
          if (lastState != SERVICE_STOPPED) {
            Thread t1;
            if (t1.Run(WaitForServiceFinishThread, nullptr) == 0 &&
                showProgressUI) {
              ShowProgressUI(true, false);
            }
            t1.Join();
          }

          lastState = WaitForServiceStop(SVC_NAME, 1);
          if (lastState != SERVICE_STOPPED) {
            // If the service doesn't stop after 10 minutes there is
            // something seriously wrong.
            lastFallbackError = FALLBACKKEY_SERVICE_NO_STOP_ERROR;
            useService = false;
          }
        } else {
          lastFallbackError = FALLBACKKEY_LAUNCH_ERROR;
        }
      }

      // If the service can't be used when staging an update, make sure that
      // the UAC prompt is not shown! In this case, just set the status to
      // pending and the update will be applied during the next startup.
      if (!useService && sStagedUpdate) {
        if (updateLockFileHandle != INVALID_HANDLE_VALUE) {
          CloseHandle(updateLockFileHandle);
        }
        WriteStatusFile("pending");
        return 0;
      }

      // If we started the service command, and it finished, check the
      // update.status file to make sure it succeeded, and if it did
      // we need to manually start the PostUpdate process from the
      // current user's session of this unelevated updater.exe the
      // current process is running as.
      // Note that we don't need to do this if we're just staging the update,
      // as the PostUpdate step runs when performing the replacing in that case.
      if (useService && !sStagedUpdate) {
        bool updateStatusSucceeded = false;
        if (IsUpdateStatusSucceeded(updateStatusSucceeded) &&
            updateStatusSucceeded) {
          if (!LaunchWinPostProcess(gInstallDirPath, gPatchDirPath)) {
            fprintf(stderr, "The post update process which runs as the user"
                    " for service update could not be launched.");
          }
        }
      }

      // If we didn't want to use the service at all, or if an update was
      // already happening, or launching the service command failed, then
      // launch the elevated updater.exe as we do without the service.
      // We don't launch the elevated updater in the case that we did have
      // write access all along because in that case the only reason we're
      // using the service is because we are testing.
      if (!useService && !noServiceFallback &&
          updateLockFileHandle == INVALID_HANDLE_VALUE) {
        SHELLEXECUTEINFO sinfo;
        memset(&sinfo, 0, sizeof(SHELLEXECUTEINFO));
        sinfo.cbSize       = sizeof(SHELLEXECUTEINFO);
        sinfo.fMask        = SEE_MASK_FLAG_NO_UI |
                             SEE_MASK_FLAG_DDEWAIT |
                             SEE_MASK_NOCLOSEPROCESS;
        sinfo.hwnd         = nullptr;
        sinfo.lpFile       = argv[0];
        sinfo.lpParameters = cmdLine;
        sinfo.lpVerb       = L"runas";
        sinfo.nShow        = SW_SHOWNORMAL;

        bool result = ShellExecuteEx(&sinfo);
        free(cmdLine);

        if (result) {
          WaitForSingleObject(sinfo.hProcess, INFINITE);
          CloseHandle(sinfo.hProcess);
        } else {
          WriteStatusFile(ELEVATION_CANCELED);
        }
      }

      if (argc > callbackIndex) {
        LaunchCallbackApp(argv[5], argc - callbackIndex,
                          argv + callbackIndex, sUsingService);
      }

      CloseHandle(elevatedFileHandle);

      if (!useService && !noServiceFallback &&
          INVALID_HANDLE_VALUE == updateLockFileHandle) {
        // We didn't use the service and we did run the elevated updater.exe.
        // The elevated updater.exe is responsible for writing out the
        // update.status file.
        return 0;
      } else if(useService) {
        // The service command was launched. The service is responsible for
        // writing out the update.status file.
        if (updateLockFileHandle != INVALID_HANDLE_VALUE) {
          CloseHandle(updateLockFileHandle);
        }
        return 0;
      } else {
        // Otherwise the service command was not launched at all.
        // We are only reaching this code path because we had write access
        // all along to the directory and a fallback key existed, and we
        // have fallback disabled (MOZ_NO_SERVICE_FALLBACK env var exists).
        // We only currently use this env var from XPCShell tests.
        CloseHandle(updateLockFileHandle);
        WriteStatusFile(lastFallbackError);
        return 0;
      }
    }
  }
#endif

#if defined(MOZ_WIDGET_GONK)
  // In gonk, the master b2g process sets its umask to 0027 because
  // there's no reason for it to ever create world-readable files.
  // The updater binary, however, needs to do this, and it inherits
  // the master process's cautious umask.  So we drop down a bit here.
  umask(0022);

  // Remount the /system partition as read-write for gonk. The destructor will
  // remount /system as read-only. We add an extra level of scope here to avoid
  // calling LogFinish() before the GonkAutoMounter destructor has a chance
  // to be called
  {
    GonkAutoMounter mounter;
    if (mounter.GetAccess() != MountAccess::ReadWrite) {
      WriteStatusFile(FILESYSTEM_MOUNT_READWRITE_ERROR);
      return 1;
    }
#endif

  if (sStagedUpdate) {
    // When staging updates, blow away the old installation directory and create
    // it from scratch.
    ensure_remove_recursive(gWorkingDirPath);
  }
  if (!sReplaceRequest) {
    // Change current directory to the directory where we need to apply the update.
    if (NS_tchdir(gWorkingDirPath) != 0) {
      // Try to create the destination directory if it doesn't exist
      int rv = NS_tmkdir(gWorkingDirPath, 0755);
      if (rv == OK && errno != EEXIST) {
        // Try changing the current directory again
        if (NS_tchdir(gWorkingDirPath) != 0) {
          // OK, time to give up!
          return 1;
        }
      } else {
        // Failed to create the directory, bail out
        return 1;
      }
    }
  }

#ifdef XP_WIN
  // For replace requests, we don't need to do any real updates, so this is not
  // necessary.
  if (!sReplaceRequest) {
    // Allocate enough space for the length of the path an optional additional
    // trailing slash and null termination.
    NS_tchar *destpath = (NS_tchar *) malloc((NS_tstrlen(gWorkingDirPath) + 2) * sizeof(NS_tchar));
    if (!destpath)
      return 1;

    NS_tchar *c = destpath;
    NS_tstrcpy(c, gWorkingDirPath);
    c += NS_tstrlen(gWorkingDirPath);
    if (gWorkingDirPath[NS_tstrlen(gWorkingDirPath) - 1] != NS_T('/') &&
        gWorkingDirPath[NS_tstrlen(gWorkingDirPath) - 1] != NS_T('\\')) {
      NS_tstrcat(c, NS_T("/"));
      c += NS_tstrlen(NS_T("/"));
    }
    *c = NS_T('\0');
    c++;

    gDestPath = destpath;
  }

  NS_tchar applyDirLongPath[MAXPATHLEN];
  if (!GetLongPathNameW(gWorkingDirPath, applyDirLongPath,
                        sizeof(applyDirLongPath)/sizeof(applyDirLongPath[0]))) {
    LOG(("NS_main: unable to find apply to dir: " LOG_S, gWorkingDirPath));
    LogFinish();
    WriteStatusFile(WRITE_ERROR);
    EXIT_WHEN_ELEVATED(elevatedLockFilePath, updateLockFileHandle, 1);
    if (argc > callbackIndex) {
      LaunchCallbackApp(argv[5], argc - callbackIndex,
                        argv + callbackIndex, sUsingService);
    }
    return 1;
  }

  HANDLE callbackFile = INVALID_HANDLE_VALUE;
  if (argc > callbackIndex) {
    // If the callback executable is specified it must exist for a successful
    // update.  It is important we null out the whole buffer here because later
    // we make the assumption that the callback application is inside the
    // apply-to dir.  If we don't have a fully null'ed out buffer it can lead
    // to stack corruption which causes crashes and other problems.
    NS_tchar callbackLongPath[MAXPATHLEN];
    ZeroMemory(callbackLongPath, sizeof(callbackLongPath));
    NS_tchar *targetPath = argv[callbackIndex];
    NS_tchar buffer[MAXPATHLEN * 2] = { NS_T('\0') };
    size_t bufferLeft = MAXPATHLEN * 2;
    if (sReplaceRequest) {
      // In case of replace requests, we should look for the callback file in
      // the destination directory.
      size_t commonPrefixLength = PathCommonPrefixW(argv[callbackIndex],
                                                    gInstallDirPath,
                                                    nullptr);
      NS_tchar *p = buffer;
      NS_tstrncpy(p, argv[callbackIndex], commonPrefixLength);
      p += commonPrefixLength;
      bufferLeft -= commonPrefixLength;
      NS_tstrncpy(p, gInstallDirPath + commonPrefixLength, bufferLeft);

      size_t len = NS_tstrlen(gInstallDirPath + commonPrefixLength);
      p += len;
      bufferLeft -= len;
      *p = NS_T('\\');
      ++p;
      bufferLeft--;
      *p = NS_T('\0');
      NS_tchar installDir[MAXPATHLEN];
      NS_tstrcpy(installDir, gInstallDirPath);
      size_t callbackPrefixLength = PathCommonPrefixW(argv[callbackIndex],
                                                      installDir,
                                                      nullptr);
      NS_tstrncpy(p, argv[callbackIndex] + std::max(callbackPrefixLength, commonPrefixLength), bufferLeft);
      targetPath = buffer;
    }
    if (!GetLongPathNameW(targetPath, callbackLongPath,
                          sizeof(callbackLongPath)/sizeof(callbackLongPath[0]))) {
      LOG(("NS_main: unable to find callback file: " LOG_S, targetPath));
      LogFinish();
      WriteStatusFile(WRITE_ERROR);
      EXIT_WHEN_ELEVATED(elevatedLockFilePath, updateLockFileHandle, 1);
      if (argc > callbackIndex) {
        LaunchCallbackApp(argv[5],
                          argc - callbackIndex,
                          argv + callbackIndex,
                          sUsingService);
      }
      return 1;
    }

    // Doing this is only necessary when we're actually applying a patch.
    if (!sReplaceRequest) {
      int len = NS_tstrlen(applyDirLongPath);
      NS_tchar *s = callbackLongPath;
      NS_tchar *d = gCallbackRelPath;
      // advance to the apply to directory and advance past the trailing backslash
      // if present.
      s += len;
      if (*s == NS_T('\\'))
        ++s;

      // Copy the string and replace backslashes with forward slashes along the
      // way.
      do {
        if (*s == NS_T('\\'))
          *d = NS_T('/');
        else
          *d = *s;
        ++s;
        ++d;
      } while (*s);
      *d = NS_T('\0');
      ++d;

      // Make a copy of the callback executable so it can be read when patching.
      NS_tsnprintf(gCallbackBackupPath,
                   sizeof(gCallbackBackupPath)/sizeof(gCallbackBackupPath[0]),
                   NS_T("%s" CALLBACK_BACKUP_EXT), argv[callbackIndex]);
      NS_tremove(gCallbackBackupPath);
      CopyFileW(argv[callbackIndex], gCallbackBackupPath, false);

      // Since the process may be signaled as exited by WaitForSingleObject before
      // the release of the executable image try to lock the main executable file
      // multiple times before giving up.  If we end up giving up, we won't
      // fail the update.
      const int max_retries = 10;
      int retries = 1;
      DWORD lastWriteError = 0;
      do {
        // By opening a file handle wihout FILE_SHARE_READ to the callback
        // executable, the OS will prevent launching the process while it is
        // being updated.
        callbackFile = CreateFileW(targetPath,
                                   DELETE | GENERIC_WRITE,
                                   // allow delete, rename, and write
                                   FILE_SHARE_DELETE | FILE_SHARE_WRITE,
                                   nullptr, OPEN_EXISTING, 0, nullptr);
        if (callbackFile != INVALID_HANDLE_VALUE)
          break;

        lastWriteError = GetLastError();
        LOG(("NS_main: callback app file open attempt %d failed. " \
             "File: " LOG_S ". Last error: %d", retries,
             targetPath, lastWriteError));

        Sleep(100);
      } while (++retries <= max_retries);

      // CreateFileW will fail if the callback executable is already in use.
      if (callbackFile == INVALID_HANDLE_VALUE) {
        // Only fail the update if the last error was not a sharing violation.
        if (lastWriteError != ERROR_SHARING_VIOLATION) {
          LOG(("NS_main: callback app file in use, failed to exclusively open " \
               "executable file: " LOG_S, argv[callbackIndex]));
          LogFinish();
          if (lastWriteError == ERROR_ACCESS_DENIED) {
            WriteStatusFile(WRITE_ERROR_ACCESS_DENIED);
          } else {
            WriteStatusFile(WRITE_ERROR_CALLBACK_APP);
          }

          NS_tremove(gCallbackBackupPath);
          EXIT_WHEN_ELEVATED(elevatedLockFilePath, updateLockFileHandle, 1);
          LaunchCallbackApp(argv[5],
                            argc - callbackIndex,
                            argv + callbackIndex,
                            sUsingService);
          return 1;
        }
        LOG(("NS_main: callback app file in use, continuing without " \
             "exclusive access for executable file: " LOG_S,
             argv[callbackIndex]));
      }
    }
  }

  // DELETE_DIR is not required when staging an update.
  if (!sStagedUpdate && !sReplaceRequest) {
    // The directory to move files that are in use to on Windows. This directory
    // will be deleted after the update is finished or on OS reboot using
    // MoveFileEx if it contains files that are in use.
    if (NS_taccess(DELETE_DIR, F_OK)) {
      NS_tmkdir(DELETE_DIR, 0755);
    }
  }
#endif /* XP_WIN */

  // Run update process on a background thread.  ShowProgressUI may return
  // before QuitProgressUI has been called, so wait for UpdateThreadFunc to
  // terminate.  Avoid showing the progress UI when staging an update.
  Thread t;
  if (t.Run(UpdateThreadFunc, nullptr) == 0) {
    if (!sStagedUpdate && !sReplaceRequest) {
      ShowProgressUI();
    }
  }
  t.Join();

#ifdef XP_WIN
  if (argc > callbackIndex && !sReplaceRequest) {
    if (callbackFile != INVALID_HANDLE_VALUE) {
      CloseHandle(callbackFile);
    }
    // Remove the copy of the callback executable.
    NS_tremove(gCallbackBackupPath);
  }

  if (!sStagedUpdate && !sReplaceRequest && _wrmdir(DELETE_DIR)) {
    LOG(("NS_main: unable to remove directory: " LOG_S ", err: %d",
         DELETE_DIR, errno));
    // The directory probably couldn't be removed due to it containing files
    // that are in use and will be removed on OS reboot. The call to remove the
    // directory on OS reboot is done after the calls to remove the files so the
    // files are removed first on OS reboot since the directory must be empty
    // for the directory removal to be successful. The MoveFileEx call to remove
    // the directory on OS reboot will fail if the process doesn't have write
    // access to the HKEY_LOCAL_MACHINE registry key but this is ok since the
    // installer / uninstaller will delete the directory along with its contents
    // after an update is applied, on reinstall, and on uninstall.
    if (MoveFileEx(DELETE_DIR, nullptr, MOVEFILE_DELAY_UNTIL_REBOOT)) {
      LOG(("NS_main: directory will be removed on OS reboot: " LOG_S,
           DELETE_DIR));
    } else {
      LOG(("NS_main: failed to schedule OS reboot removal of " \
           "directory: " LOG_S, DELETE_DIR));
    }
  }
#endif /* XP_WIN */

#if defined(MOZ_WIDGET_GONK)
  } // end the extra level of scope for the GonkAutoMounter
#endif

#ifdef XP_MACOSX
  // When the update is successful remove the precomplete file in the root of
  // the application bundle and move the distribution directory from
  // Contents/MacOS to Contents/Resources and if both exist delete the
  // directory under Contents/MacOS (see Bug 1068439).
  if (gSucceeded && !sStagedUpdate) {
    NS_tchar oldPrecomplete[MAXPATHLEN];
    NS_tsnprintf(oldPrecomplete, sizeof(oldPrecomplete)/sizeof(oldPrecomplete[0]),
                 NS_T("%s/precomplete"), gInstallDirPath);
    NS_tremove(oldPrecomplete);

    NS_tchar oldDistDir[MAXPATHLEN];
    NS_tsnprintf(oldDistDir, sizeof(oldDistDir)/sizeof(oldDistDir[0]),
                 NS_T("%s/Contents/MacOS/distribution"), gInstallDirPath);
    int rv = NS_taccess(oldDistDir, F_OK);
    if (!rv) {
      NS_tchar newDistDir[MAXPATHLEN];
      NS_tsnprintf(newDistDir, sizeof(newDistDir)/sizeof(newDistDir[0]),
                   NS_T("%s/Contents/Resources/distribution"), gInstallDirPath);
      rv = NS_taccess(newDistDir, F_OK);
      if (!rv) {
        LOG(("New distribution directory already exists... removing old " \
             "distribution directory: " LOG_S, oldDistDir));
        rv = ensure_remove_recursive(oldDistDir);
        if (rv) {
          LOG(("Removing old distribution directory failed - err: %d", rv));
        }
      } else {
        LOG(("Moving old distribution directory to new location. src: " LOG_S \
             ", dst:" LOG_S, oldDistDir, newDistDir));
        rv = rename_file(oldDistDir, newDistDir, true);
        if (rv) {
          LOG(("Moving old distribution directory to new location failed - " \
               "err: %d", rv));
        }
      }
    }
  }
#endif /* XP_MACOSX */

  LogFinish();

  if (argc > callbackIndex) {
#if defined(XP_WIN)
    if (gSucceeded) {
      if (!LaunchWinPostProcess(gInstallDirPath, gPatchDirPath)) {
        fprintf(stderr, "The post update process was not launched");
      }

      // The service update will only be executed if it is already installed.
      // For first time installs of the service, the install will happen from
      // the PostUpdate process. We do the service update process here
      // because it's possible we are updating with updater.exe without the
      // service if the service failed to apply the update. We want to update
      // the service to a newer version in that case. If we are not running
      // through the service, then MOZ_USING_SERVICE will not exist.
      if (!sUsingService) {
        StartServiceUpdate(gInstallDirPath);
      }
    }
    EXIT_WHEN_ELEVATED(elevatedLockFilePath, updateLockFileHandle, 0);
#endif /* XP_WIN */
#ifdef XP_MACOSX
    if (gSucceeded) {
      LaunchMacPostProcess(gInstallDirPath);
    }
#endif /* XP_MACOSX */

    if (getenv("MOZ_PROCESS_UPDATES") == nullptr) {
      LaunchCallbackApp(argv[5],
                        argc - callbackIndex,
                        argv + callbackIndex,
                        sUsingService);
    }
  }

  return gSucceeded ? 0 : 1;
}

class ActionList
{
public:
  ActionList() : mFirst(nullptr), mLast(nullptr), mCount(0) { }
  ~ActionList();

  void Append(Action* action);
  int Prepare();
  int Execute();
  void Finish(int status);

private:
  Action *mFirst;
  Action *mLast;
  int     mCount;
};

ActionList::~ActionList()
{
  Action* a = mFirst;
  while (a) {
    Action *b = a;
    a = a->mNext;
    delete b;
  }
}

void
ActionList::Append(Action *action)
{
  if (mLast)
    mLast->mNext = action;
  else
    mFirst = action;

  mLast = action;
  mCount++;
}

int
ActionList::Prepare()
{
  // If the action list is empty then we should fail in order to signal that
  // something has gone wrong. Otherwise we report success when nothing is
  // actually done. See bug 327140.
  if (mCount == 0) {
    LOG(("empty action list"));
    return UNEXPECTED_MAR_ERROR;
  }

  Action *a = mFirst;
  int i = 0;
  while (a) {
    int rv = a->Prepare();
    if (rv)
      return rv;

    float percent = float(++i) / float(mCount);
    UpdateProgressUI(PROGRESS_PREPARE_SIZE * percent);

    a = a->mNext;
  }

  return OK;
}

int
ActionList::Execute()
{
  int currentProgress = 0, maxProgress = 0;
  Action *a = mFirst;
  while (a) {
    maxProgress += a->mProgressCost;
    a = a->mNext;
  }

  a = mFirst;
  while (a) {
    int rv = a->Execute();
    if (rv) {
      LOG(("### execution failed"));
      return rv;
    }

    currentProgress += a->mProgressCost;
    float percent = float(currentProgress) / float(maxProgress);
    UpdateProgressUI(PROGRESS_PREPARE_SIZE +
                     PROGRESS_EXECUTE_SIZE * percent);

    a = a->mNext;
  }

  return OK;
}

void
ActionList::Finish(int status)
{
  Action *a = mFirst;
  int i = 0;
  while (a) {
    a->Finish(status);

    float percent = float(++i) / float(mCount);
    UpdateProgressUI(PROGRESS_PREPARE_SIZE +
                     PROGRESS_EXECUTE_SIZE +
                     PROGRESS_FINISH_SIZE * percent);

    a = a->mNext;
  }

  if (status == OK)
    gSucceeded = true;
}


#ifdef XP_WIN
int add_dir_entries(const NS_tchar *dirpath, ActionList *list)
{
  int rv = OK;
  WIN32_FIND_DATAW finddata;
  HANDLE hFindFile;
  NS_tchar searchspec[MAXPATHLEN];
  NS_tchar foundpath[MAXPATHLEN];

  NS_tsnprintf(searchspec, sizeof(searchspec)/sizeof(searchspec[0]),
               NS_T("%s*"), dirpath);
  const NS_tchar *pszSpec = get_full_path(searchspec);

  hFindFile = FindFirstFileW(pszSpec, &finddata);
  if (hFindFile != INVALID_HANDLE_VALUE) {
    do {
      // Don't process the current or parent directory.
      if (NS_tstrcmp(finddata.cFileName, NS_T(".")) == 0 ||
          NS_tstrcmp(finddata.cFileName, NS_T("..")) == 0)
        continue;

      NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                   NS_T("%s%s"), dirpath, finddata.cFileName);
      if (finddata.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) {
        NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                     NS_T("%s/"), foundpath);
        // Recurse into the directory.
        rv = add_dir_entries(foundpath, list);
        if (rv) {
          LOG(("add_dir_entries error: " LOG_S ", err: %d", foundpath, rv));
          return rv;
        }
      } else {
        // Add the file to be removed to the ActionList.
        NS_tchar *quotedpath = get_quoted_path(foundpath);
        if (!quotedpath)
          return PARSE_ERROR;

        Action *action = new RemoveFile();
        rv = action->Parse(quotedpath);
        if (rv) {
          LOG(("add_dir_entries Parse error on recurse: " LOG_S ", err: %d", quotedpath, rv));
          return rv;
        }

        list->Append(action);
      }
    } while (FindNextFileW(hFindFile, &finddata) != 0);

    FindClose(hFindFile);
    {
      // Add the directory to be removed to the ActionList.
      NS_tchar *quotedpath = get_quoted_path(dirpath);
      if (!quotedpath)
        return PARSE_ERROR;

      Action *action = new RemoveDir();
      rv = action->Parse(quotedpath);
      if (rv)
        LOG(("add_dir_entries Parse error on close: " LOG_S ", err: %d", quotedpath, rv));
      else
        list->Append(action);
    }
  }

  return rv;
}

#elif defined(SOLARIS)
int add_dir_entries(const NS_tchar *dirpath, ActionList *list)
{
  int rv = OK;
  NS_tchar searchpath[MAXPATHLEN];
  NS_tchar foundpath[MAXPATHLEN];
  struct {
    dirent dent_buffer;
    char chars[MAXNAMLEN];
  } ent_buf;
  struct dirent* ent;


  NS_tsnprintf(searchpath, sizeof(searchpath)/sizeof(searchpath[0]), NS_T("%s"),
               dirpath);
  // Remove the trailing slash so the paths don't contain double slashes. The
  // existence of the slash has already been checked in DoUpdate.
  searchpath[NS_tstrlen(searchpath) - 1] = NS_T('\0');

  DIR* dir = opendir(searchpath);
  if (!dir) {
    LOG(("add_dir_entries error on opendir: " LOG_S ", err: %d", searchpath,
         errno));
    return UNEXPECTED_FILE_OPERATION_ERROR;
  }

  while (readdir_r(dir, (dirent *)&ent_buf, &ent) == 0 && ent) {
    if ((strcmp(ent->d_name, ".") == 0) ||
        (strcmp(ent->d_name, "..") == 0))
      continue;

    NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                 NS_T("%s%s"), dirpath, ent->d_name);
    struct stat64 st_buf;
    int test = stat64(foundpath, &st_buf);
    if (test) {
      closedir(dir);
      return UNEXPECTED_FILE_OPERATION_ERROR;
    }
    if (S_ISDIR(st_buf.st_mode)) {
      NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                   NS_T("%s/"), foundpath);
      // Recurse into the directory.
      rv = add_dir_entries(foundpath, list);
      if (rv) {
        LOG(("add_dir_entries error: " LOG_S ", err: %d", foundpath, rv));
        closedir(dir);
        return rv;
      }
    } else {
      // Add the file to be removed to the ActionList.
      NS_tchar *quotedpath = get_quoted_path(foundpath);
      if (!quotedpath) {
        closedir(dir);
        return PARSE_ERROR;
      }

      Action *action = new RemoveFile();
      rv = action->Parse(quotedpath);
      if (rv) {
        LOG(("add_dir_entries Parse error on recurse: " LOG_S ", err: %d",
             quotedpath, rv));
        closedir(dir);
        return rv;
      }

      list->Append(action);
    }
  }
  closedir(dir);

  // Add the directory to be removed to the ActionList.
  NS_tchar *quotedpath = get_quoted_path(dirpath);
  if (!quotedpath)
    return PARSE_ERROR;

  Action *action = new RemoveDir();
  rv = action->Parse(quotedpath);
  if (rv) {
    LOG(("add_dir_entries Parse error on close: " LOG_S ", err: %d",
         quotedpath, rv));
  }
  else {
    list->Append(action);
  }

  return rv;
}

#else

int add_dir_entries(const NS_tchar *dirpath, ActionList *list)
{
  int rv = OK;
  FTS *ftsdir;
  FTSENT *ftsdirEntry;
  NS_tchar searchpath[MAXPATHLEN];

  NS_tsnprintf(searchpath, sizeof(searchpath)/sizeof(searchpath[0]), NS_T("%s"),
               dirpath);
  // Remove the trailing slash so the paths don't contain double slashes. The
  // existence of the slash has already been checked in DoUpdate.
  searchpath[NS_tstrlen(searchpath) - 1] = NS_T('\0');
  char* const pathargv[] = {searchpath, nullptr};

  // FTS_NOCHDIR is used so relative paths from the destination directory are
  // returned.
  if (!(ftsdir = fts_open(pathargv,
                          FTS_PHYSICAL | FTS_NOSTAT | FTS_XDEV | FTS_NOCHDIR,
                          nullptr)))
    return UNEXPECTED_FILE_OPERATION_ERROR;

  while ((ftsdirEntry = fts_read(ftsdir)) != nullptr) {
    NS_tchar foundpath[MAXPATHLEN];
    NS_tchar *quotedpath;
    Action *action = nullptr;

    switch (ftsdirEntry->fts_info) {
      // Filesystem objects that shouldn't be in the application's directories
      case FTS_SL:
      case FTS_SLNONE:
      case FTS_DEFAULT:
        LOG(("add_dir_entries: found a non-standard file: " LOG_S,
             ftsdirEntry->fts_path));
        // Fall through and try to remove as a file

      // Files
      case FTS_F:
      case FTS_NSOK:
        // Add the file to be removed to the ActionList.
        NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                     NS_T("%s"), ftsdirEntry->fts_accpath);
        quotedpath = get_quoted_path(foundpath);
        if (!quotedpath) {
          rv = UPDATER_QUOTED_PATH_MEM_ERROR;
          break;
        }
        action = new RemoveFile();
        rv = action->Parse(quotedpath);
        if (!rv)
          list->Append(action);
        break;

      // Directories
      case FTS_DP:
        rv = OK;
        // Add the directory to be removed to the ActionList.
        NS_tsnprintf(foundpath, sizeof(foundpath)/sizeof(foundpath[0]),
                     NS_T("%s/"), ftsdirEntry->fts_accpath);
        quotedpath = get_quoted_path(foundpath);
        if (!quotedpath) {
          rv = UPDATER_QUOTED_PATH_MEM_ERROR;
          break;
        }

        action = new RemoveDir();
        rv = action->Parse(quotedpath);
        if (!rv)
          list->Append(action);
        break;

      // Errors
      case FTS_DNR:
      case FTS_NS:
        // ENOENT is an acceptable error for FTS_DNR and FTS_NS and means that
        // we're racing with ourselves. Though strange, the entry will be
        // removed anyway.
        if (ENOENT == ftsdirEntry->fts_errno) {
          rv = OK;
          break;
        }
        // Fall through

      case FTS_ERR:
        rv = UNEXPECTED_FILE_OPERATION_ERROR;
        LOG(("add_dir_entries: fts_read() error: " LOG_S ", err: %d",
             ftsdirEntry->fts_path, ftsdirEntry->fts_errno));
        break;

      case FTS_DC:
        rv = UNEXPECTED_FILE_OPERATION_ERROR;
        LOG(("add_dir_entries: fts_read() returned FT_DC: " LOG_S,
             ftsdirEntry->fts_path));
        break;

      default:
        // FTS_D is ignored and FTS_DP is used instead (post-order).
        rv = OK;
        break;
    }

    if (rv != OK)
      break;
  }

  fts_close(ftsdir);

  return rv;
}
#endif

static NS_tchar*
GetManifestContents(const NS_tchar *manifest)
{
  AutoFile mfile(NS_tfopen(manifest, NS_T("rb")));
  if (mfile == nullptr) {
    LOG(("GetManifestContents: error opening manifest file: " LOG_S, manifest));
    return nullptr;
  }

  struct stat ms;
  int rv = fstat(fileno((FILE *)mfile), &ms);
  if (rv) {
    LOG(("GetManifestContents: error stating manifest file: " LOG_S, manifest));
    return nullptr;
  }

  char *mbuf = (char *) malloc(ms.st_size + 1);
  if (!mbuf)
    return nullptr;

  size_t r = ms.st_size;
  char *rb = mbuf;
  while (r) {
    const size_t count = mmin(SSIZE_MAX, r);
    size_t c = fread(rb, 1, count, mfile);
    if (c != count) {
      LOG(("GetManifestContents: error reading manifest file: " LOG_S, manifest));
      return nullptr;
    }

    r -= c;
    rb += c;
  }
  mbuf[ms.st_size] = '\0';
  rb = mbuf;

#ifndef XP_WIN
  return rb;
#else
  NS_tchar *wrb = (NS_tchar *) malloc((ms.st_size + 1) * sizeof(NS_tchar));
  if (!wrb)
    return nullptr;

  if (!MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, rb, -1, wrb,
                           ms.st_size + 1)) {
    LOG(("GetManifestContents: error converting utf8 to utf16le: %d", GetLastError()));
    free(mbuf);
    free(wrb);
    return nullptr;
  }
  free(mbuf);

  return wrb;
#endif
}

int AddPreCompleteActions(ActionList *list)
{
  if (sIsOSUpdate) {
    return OK;
  }

#ifdef XP_MACOSX
  NS_tchar *rb = GetManifestContents(NS_T("Contents/Resources/precomplete"));
#else
  NS_tchar *rb = GetManifestContents(NS_T("precomplete"));
#endif
  if (rb == nullptr) {
    LOG(("AddPreCompleteActions: error getting contents of precomplete " \
         "manifest"));
    // Applications aren't required to have a precomplete manifest. The mar
    // generation scripts enforce the presence of a precomplete manifest.
    return OK;
  }

  int rv;
  NS_tchar *line;
  while((line = mstrtok(kNL, &rb)) != 0) {
    // skip comments
    if (*line == NS_T('#'))
      continue;

    NS_tchar *token = mstrtok(kWhitespace, &line);
    if (!token) {
      LOG(("AddPreCompleteActions: token not found in manifest"));
      return PARSE_ERROR;
    }

    Action *action = nullptr;
    if (NS_tstrcmp(token, NS_T("remove")) == 0) { // rm file
      action = new RemoveFile();
    }
    else if (NS_tstrcmp(token, NS_T("remove-cc")) == 0) { // no longer supported
      continue;
    }
    else if (NS_tstrcmp(token, NS_T("rmdir")) == 0) { // rmdir if  empty
      action = new RemoveDir();
    }
    else {
      LOG(("AddPreCompleteActions: unknown token: " LOG_S, token));
      return PARSE_ERROR;
    }

    if (!action)
      return BAD_ACTION_ERROR;

    rv = action->Parse(line);
    if (rv)
      return rv;

    list->Append(action);
  }

  return OK;
}

int DoUpdate()
{
  NS_tchar manifest[MAXPATHLEN];
  NS_tsnprintf(manifest, sizeof(manifest)/sizeof(manifest[0]),
               NS_T("%s/updating/update.manifest"), gWorkingDirPath);
  ensure_parent_dir(manifest);

  // extract the manifest
  int rv = gArchiveReader.ExtractFile("updatev3.manifest", manifest);
  if (rv) {
    rv = gArchiveReader.ExtractFile("updatev2.manifest", manifest);
    if (rv) {
      LOG(("DoUpdate: error extracting manifest file"));
      return rv;
    }
  }

  NS_tchar *rb = GetManifestContents(manifest);
  NS_tremove(manifest);
  if (rb == nullptr) {
    LOG(("DoUpdate: error opening manifest file: " LOG_S, manifest));
    return READ_ERROR;
  }


  ActionList list;
  NS_tchar *line;
  bool isFirstAction = true;

  while((line = mstrtok(kNL, &rb)) != 0) {
    // skip comments
    if (*line == NS_T('#'))
      continue;

    NS_tchar *token = mstrtok(kWhitespace, &line);
    if (!token) {
      LOG(("DoUpdate: token not found in manifest"));
      return PARSE_ERROR;
    }

    if (isFirstAction) {
      isFirstAction = false;
      // The update manifest isn't required to have a type declaration. The mar
      // generation scripts enforce the presence of the type declaration.
      if (NS_tstrcmp(token, NS_T("type")) == 0) {
        const NS_tchar *type = mstrtok(kQuote, &line);
        LOG(("UPDATE TYPE " LOG_S, type));
        if (NS_tstrcmp(type, NS_T("complete")) == 0) {
          rv = AddPreCompleteActions(&list);
          if (rv)
            return rv;
        }
        continue;
      }
    }

    Action *action = nullptr;
    if (NS_tstrcmp(token, NS_T("remove")) == 0) { // rm file
      action = new RemoveFile();
    }
    else if (NS_tstrcmp(token, NS_T("rmdir")) == 0) { // rmdir if empty
      action = new RemoveDir();
    }
    else if (NS_tstrcmp(token, NS_T("rmrfdir")) == 0) { // rmdir recursive
      const NS_tchar *reldirpath = mstrtok(kQuote, &line);
      if (!reldirpath)
        return PARSE_ERROR;

      if (reldirpath[NS_tstrlen(reldirpath) - 1] != NS_T('/'))
        return PARSE_ERROR;

      rv = add_dir_entries(reldirpath, &list);
      if (rv)
        return rv;

      continue;
    }
    else if (NS_tstrcmp(token, NS_T("add")) == 0) {
      action = new AddFile();
    }
    else if (NS_tstrcmp(token, NS_T("patch")) == 0) {
      action = new PatchFile();
    }
    else if (NS_tstrcmp(token, NS_T("add-if")) == 0) { // Add if exists
      action = new AddIfFile();
    }
    else if (NS_tstrcmp(token, NS_T("add-if-not")) == 0) { // Add if not exists
      action = new AddIfNotFile();
    }
    else if (NS_tstrcmp(token, NS_T("patch-if")) == 0) { // Patch if exists
      action = new PatchIfFile();
    }
    else {
      LOG(("DoUpdate: unknown token: " LOG_S, token));
      return PARSE_ERROR;
    }

    if (!action)
      return BAD_ACTION_ERROR;

    rv = action->Parse(line);
    if (rv)
      return rv;

    list.Append(action);
  }

  rv = list.Prepare();
  if (rv)
    return rv;

  rv = list.Execute();

  list.Finish(rv);
  return rv;
}