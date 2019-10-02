const char crurom_rcsid[] = "@(#) $Id: crurom.c 4392 2012-07-24 10:18:44Z haraldkipp $";

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#ifdef _WIN32
#include <io.h>
#include "dirent.h"
#else
#include <unistd.h>
#include <dirent.h>
#define stricmp strcasecmp
#define strnicmp strncasecmp
#endif

#include "getopt.h"

#ifndef O_BINARY
#define O_BINARY 0
#endif


#define IDENT   "crurom"
#undef VERSION
#define VERSION "1.3.3"

static int entryno = 0;
static int verbose = 0;
static int recursive = 0;
static char rootdir[256];
static int rootlen = 0;
static char outname[256];
static FILE *fpout;

int dofile(char *name)
{
    int rc = 0;
    int fd;
    unsigned char buf[512];
    int i;
    int cnt;
    long total = 0;
    char *fsname = name;

    if(strnicmp(fsname, rootdir, rootlen) == 0)
        fsname += rootlen;

    if((fd = open(name, O_RDONLY | O_BINARY)) == -1) {
        perror(name);
        return -1;
    }
    if(verbose)
        fprintf(stderr, IDENT ": Reading %s\n", name);

    for(;;) {
        if((cnt = read(fd, buf, sizeof(buf))) < 0) {
            perror(name);
            rc = -1;
            total = 0;
            break;
        }
        if(total == 0) {
            entryno++;
            fprintf(fpout, "/*\n * File entry %d: %s\n */\n", entryno, fsname);
            fprintf(fpout, "prog_char file%ddata[] = {", entryno);
        }
        if(cnt == 0)
            break;
        for(i = 0; i < cnt; i++) {
            if((i % 16) == 0) {
                if(total != 0 || i != 0) {
                    fputc(',', fpout);
                }
                fputs("\n ", fpout);
            } else {
                fputc(',', fpout);
            }
            if (buf[i] < 32 || buf[i] > 127 || buf[i] == '\'' || buf[i] == '\\') {
                fprintf(fpout, "%3u", buf[i]);
            }
            else
                fprintf(fpout, "'%c'", buf[i]);
        }
        total += cnt;
    }
    close(fd);

    fprintf(fpout, "\n};\n\n");

    fprintf(fpout, "prog_char file%dname[] = \"%s\";\n\n", entryno, fsname);

    fprintf(fpout, "static ROMENTRY file%dentry = { ", entryno);

    if(entryno > 1)
        fprintf(fpout, "&file%dentry, ", entryno - 1);
    else
        fprintf(fpout, "0, ");

    fprintf(fpout, "(prog_char *)file%dname, %ld, (prog_char *)file%ddata };\n", entryno, total, entryno);

    return rc;
}

int dodir(char *dirpath)
{
    int rc = 0;
    char path[256];
    DIR *dir;
    struct dirent *dire;
    struct stat statbuf;

    if((dir = opendir(dirpath)) == NULL) {
        fprintf(stderr, "Failed to scan directory %s\n", dirpath);
        return -1;
    }
    if(verbose)
        fprintf(stderr, "Scan %s\n", dirpath);
    while((dire = readdir(dir)) != NULL && rc == 0) {
        if((dire->d_name[0] == '.') || (stricmp(dire->d_name, "cvs") == 0) || (stricmp(dire->d_name, "svn") == 0))
            continue;
        strcpy(path, dirpath);
        strcat(path, "/");
        strcat(path, dire->d_name);
        stat(path, &statbuf);

        if(statbuf.st_mode & S_IFDIR)
            rc = dodir(path);
        else if(statbuf.st_mode & S_IFREG)
            rc = dofile(path);
    }
    closedir(dir);
    return rc;
}

void usage(void)
{
    fputs("Usage: crurom OPTIONS DIRECTORY\n"
      "OPTIONS:\n"
      "-ofile  output file\n"
      "-r      recursive\n"
      "-v      verbose\n"
    , stderr);
}

int main(int argc, char **argv)
{
    int option;
    int i;
    int rc = 0;

    while((option = getopt(argc, argv, "o:rv?")) != EOF) {
        switch(option) {
        case 'o':
            strcpy(outname, optarg);
            break;
        case 'r':
            recursive++;
            break;
        case 'v':
            verbose++;
            break;
        default:
            usage();
            return 1;
        }
    }
    argc -= optind;
    argv += optind;

    if(outname[0]) {
        if((fpout = fopen(outname, "w")) == NULL) {
            perror(outname);
            return 3;
        }
    }
    else
        fpout = stdout;

    fprintf(fpout, "/*\n");
    fprintf(fpout, " * This file is automatically created by " IDENT " " VERSION "\n");
    fprintf(fpout, " */\n");
    fprintf(fpout, "#include <fs/uromfs.h>\n\n");

    if(argc) {
        for(i = 0; i < argc && rc == 0; i++) {
            strcpy(rootdir, argv[i]);
            strcat(rootdir, "/");
            rootlen = strlen(rootdir);
            rc = dodir(argv[i]);
        }
    }
    else {
        strcpy(rootdir, "./");
        rootlen = 2;
        rc = dodir(".");
    }
    fprintf(fpout, "\nROMENTRY *romEntryList = &file%dentry;\n", entryno);
    if(fpout != stdout)
        fclose(fpout);
    return rc;
}
