/*
 * a2091test - file level write/read/verify through a (SCSI) volume
 *
 *   a2091test <dir> [KB] [passes]
 *
 * For each pass and for chip and fast/any memory buffers: write a KB-sized
 * pseudo-random file to <dir>, read it back into a second buffer of the same
 * memory type, compare, and print the transfer rates. Also does a series of
 * odd-sized reads at odd offsets (exercises the bounce paths of the driver).
 *
 * Build: m68k-amigaos-gcc -O2 -mcrt=nix20 -o a2091test a2091test.c
 */
#include <exec/types.h>
#include <exec/memory.h>
#include <dos/dos.h>
#include <proto/exec.h>
#include <proto/dos.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static ULONG seed;
static ULONG rnd(void)
{
	seed = seed * 1103515245UL + 12345UL;
	return seed;
}

static ULONG ticks(void)
{
	struct DateStamp ds;
	DateStamp(&ds);
	return (ULONG)ds.ds_Minute * 3000UL + (ULONG)ds.ds_Tick;
}

static void rate(const char *what, ULONG bytes, ULONG t)
{
	if (!t) t = 1;
	ULONG kbs = (bytes / 1024UL) * 50UL / t;
	printf("  %-6s %6u KB in %4u ticks = %5u KB/s\n", what, bytes / 1024UL, t, kbs);
}

static int onepass(const char *path, ULONG size, ULONG memtype, const char *mname, int pass)
{
	UBYTE *a = AllocMem(size, memtype);
	UBYTE *b = AllocMem(size, memtype);
	int ok = 0;
	BPTR fh;
	ULONG i, t;

	if (!a || !b)
	{
		printf("%s: cannot allocate 2 x %u bytes\n", mname, size);
		goto out;
	}
	printf("pass %d, %s buffers @ $%08x / $%08x\n", pass, mname, (ULONG)a, (ULONG)b);

	seed = 0x2091 + pass * 77 + (memtype & MEMF_CHIP ? 1 : 0);
	for (i = 0; i < size; i += 4) *(ULONG *)(a + i) = rnd();
	memset(b, 0xee, size);

	fh = Open(path, MODE_NEWFILE);
	if (!fh) { printf("  cannot create %s (%d)\n", path, IoErr()); goto out; }
	t = ticks();
	if (Write(fh, a, size) != (LONG)size) { printf("  write failed (%d)\n", IoErr()); Close(fh); goto out; }
	Close(fh);
	rate("write", size, ticks() - t);

	fh = Open(path, MODE_OLDFILE);
	if (!fh) { printf("  cannot open %s (%d)\n", path, IoErr()); goto out; }
	t = ticks();
	if (Read(fh, b, size) != (LONG)size) { printf("  read failed (%d)\n", IoErr()); Close(fh); goto out; }
	rate("read", size, ticks() - t);

	for (i = 0; i < size; i++)
	{
		if (a[i] != b[i])
		{
			printf("  MISMATCH at %u: wrote %02x read %02x\n", i, a[i], b[i]);
			Close(fh);
			goto out;
		}
	}

	/* odd sized / odd offset reads */
	for (i = 0; i < 16; i++)
	{
		ULONG off = (rnd() >> 8) % (size - 3000);
		ULONG len = 1 + (rnd() >> 8) % 2999;
		ULONG k;
		Seek(fh, off, OFFSET_BEGINNING);
		memset(b + 1, 0x55, len);
		if (Read(fh, b + 1, len) != (LONG)len) { printf("  short read at %u/%u\n", off, len); Close(fh); goto out; }
		for (k = 0; k < len; k++)
			if (b[1 + k] != a[off + k])
			{
				printf("  MISMATCH odd read off %u len %u at %u\n", off, len, k);
				Close(fh);
				goto out;
			}
	}
	Close(fh);
	printf("  verify OK\n");
	ok = 1;

out:
	if (a) FreeMem(a, size);
	if (b) FreeMem(b, size);
	return ok;
}

int main(int argc, char **argv)
{
	char path[256];
	ULONG kb = argc > 2 ? (ULONG)atoi(argv[2]) : 256;
	int passes = argc > 3 ? atoi(argv[3]) : 1;
	int p, fails = 0;

	if (argc < 2)
	{
		printf("usage: a2091test <dir> [KB] [passes]\n");
		return 5;
	}
	strncpy(path, argv[1], sizeof(path) - 16);
	path[sizeof(path) - 16] = 0;
	if (path[0] && path[strlen(path) - 1] != ':' && path[strlen(path) - 1] != '/') strcat(path, "/");
	strcat(path, "a2091t.bin");

	if (kb < 4) kb = 4;
	printf("a2091test: %s, %u KB, %d pass(es)\n", path, kb, passes);
	for (p = 1; p <= passes; p++)
	{
		if (!onepass(path, kb * 1024UL, MEMF_CHIP, "chip", p)) fails++;
		if (!onepass(path, kb * 1024UL, MEMF_FAST, "fast", p)) fails++;
	}
	DeleteFile(path);
	printf("a2091test: %s\n", fails ? "FAILED" : "PASSED");
	return fails ? 10 : 0;
}
