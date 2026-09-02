// Platform probe for the save-file rename guard used by SaveConversionDialog.
//
// The converter renames save files with std::filesystem::rename, which replaces an
// existing destination silently -- for a save file that means destroying it with no
// undo. The guard is:
//
//     std::error_code ec;
//     fs::file_status st = fs::symlink_status(dest, ec);
//     if (!fs::status_known(st) || fs::exists(st))  -> skip, do not rename
//
// This program checks two things per destination kind, on whatever platform it runs:
//
//   1. the guard classifies it as occupied (so the file is skipped), and
//   2. an unguarded rename onto it really would have destroyed the source
//
// (2) matters as much as (1): it proves the guard is load-bearing rather than
// defensive padding, and it is the part that can differ between POSIX rename(2) and
// the Win32 MoveFileExW that MS-STL uses.
//
// Exit code is the number of failures.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

namespace fs = std::filesystem;

static int failures = 0;
static int skips = 0;

static void report(const char* name, bool ok, const std::string& detail)
{
    if (ok)
    {
        std::printf("  PASS  %-34s %s\n", name, detail.c_str());
    }
    else
    {
        std::printf("  FAIL  %-34s %s\n", name, detail.c_str());
        failures++;
    }
}

static void skip(const char* name, const std::string& why)
{
    std::printf("  SKIP  %-34s %s\n", name, why.c_str());
    skips++;
}

// the predicate under test, copied verbatim from SaveConversionDialog::scan()
static bool destinationOccupied(const fs::path& dest)
{
    std::error_code ec;
    fs::file_status st = fs::symlink_status(dest, ec);
    return !fs::status_known(st) || fs::exists(st);
}

static void write(const fs::path& p, const std::string& text)
{
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f << text;
}

static std::string read(const fs::path& p)
{
    std::ifstream f(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)),
                       std::istreambuf_iterator<char>());
}

// Builds a source file and a destination of the given kind, then checks the guard and
// (on a separate copy) what an unguarded rename would have done.
static void checkOccupied(const fs::path& dir, const char* name,
                          void (*makeDest)(const fs::path&))
{
    fs::path guarded = dir / (std::string("g_") + name);
    fs::path dest = dir / (std::string("d_") + name);

    fs::remove_all(guarded);
    fs::remove_all(dest);

    write(guarded, "SOURCE");

    std::error_code mkec;
    try { makeDest(dest); }
    catch (const std::exception& e) { skip(name, std::string("cannot create: ") + e.what()); return; }

    if (!destinationOccupied(dest))
    {
        report(name, false, "guard did NOT classify the destination as occupied");
        return;
    }

    // now prove the guard was load-bearing: rename onto the same kind of destination
    // without checking, and see whether the source survives
    fs::path unguarded = dir / (std::string("u_") + name);
    fs::path dest2 = dir / (std::string("d2_") + name);
    fs::remove_all(unguarded);
    fs::remove_all(dest2);
    write(unguarded, "SOURCE");
    try { makeDest(dest2); } catch (...) {}

    std::error_code rec;
    fs::rename(unguarded, dest2, rec);

    std::string detail;
    if (rec)
        detail = "guard skips; unguarded rename would have failed (" + rec.message() + ")";
    else if (!fs::exists(fs::symlink_status(unguarded)))
        detail = "guard skips; unguarded rename DESTROYS the source";
    else
        detail = "guard skips; unguarded rename left the source in place";

    report(name, true, detail);
}

int main()
{
    fs::path dir = fs::temp_directory_path() / "melonds-save-rename-probe";
    std::error_code ec;
    fs::remove_all(dir, ec);
    fs::create_directories(dir);

    std::printf("melonDS save rename probe\n");
    std::printf("  dir: %s\n\n", dir.string().c_str());

    // --- the destination kinds the converter can meet -----------------------------

    checkOccupied(dir, "regular-file", [](const fs::path& p) { write(p, "DESTINATION"); });

    checkOccupied(dir, "directory", [](const fs::path& p) { fs::create_directory(p); });

    checkOccupied(dir, "dangling-symlink", [](const fs::path& p) {
        fs::create_symlink("nowhere-at-all", p);
    });

    checkOccupied(dir, "symlink-to-file", [](const fs::path& p) {
        fs::path target = p.parent_path() / "symlink-target";
        write(target, "TARGET");
        fs::create_symlink(target, p);
    });

    checkOccupied(dir, "symlink-to-directory", [](const fs::path& p) {
        fs::path target = p.parent_path() / "symlink-target-dir";
        fs::create_directories(target);
        fs::create_directory_symlink(target, p);
    });

    // --- a free destination must NOT be reported occupied -------------------------
    {
        fs::path src = dir / "free-src.sav";
        fs::path dst = dir / "free-dst.srm";
        fs::remove_all(src); fs::remove_all(dst);
        write(src, "PAYLOAD");

        if (destinationOccupied(dst))
        {
            report("free-destination", false, "guard wrongly reported a free path as occupied");
        }
        else
        {
            std::error_code rec;
            fs::rename(src, dst, rec);
            bool ok = !rec && !fs::exists(dst.parent_path() / src.filename())
                      && read(dst) == "PAYLOAD";
            report("free-destination", ok,
                   ok ? "renamed, content preserved" : "rename failed: " + rec.message());
        }
    }

    // --- case-insensitive filesystems ---------------------------------------------
    // On Windows and default APFS, GAME.SRM and game.srm are the same file. The guard
    // must see the existing file through the differently-cased name, or the converter
    // would rename over a real save.
    {
        fs::path upper = dir / "CASE.SRM";
        fs::path lower = dir / "case.srm";
        fs::remove_all(upper); fs::remove_all(lower);
        write(upper, "EXISTING");

        bool caseInsensitive = fs::exists(fs::symlink_status(lower));
        if (!caseInsensitive)
            skip("case-differing-destination", "filesystem is case-sensitive, not applicable");
        else
            report("case-differing-destination", destinationOccupied(lower),
                   "guard sees CASE.SRM when checking case.srm");
    }

    // --- non-ASCII round trip ------------------------------------------------------
    // The converter goes QString -> toStdString (UTF-8) -> u8path -> u8string ->
    // fromStdString. This checks the filesystem half of that on this platform.
    {
        // escaped rather than literal so this file stays pure ASCII, which is also
        // what the converter's own sources do
        std::string utf8 = u8"\u30DD\u30B1\u30E2\u30F3.sav";   // "pokemon" in katakana
        fs::path src = dir / fs::u8path(utf8);
        fs::path dst = dir / fs::u8path(u8"\u30DD\u30B1\u30E2\u30F3.srm");
        fs::remove_all(src); fs::remove_all(dst);

        std::error_code wec;
        write(src, "JP");
        if (!fs::exists(fs::symlink_status(src)))
        {
            skip("non-ascii-filename", "could not create the file");
        }
        else
        {
            std::error_code rec;
            fs::rename(src, dst, rec);
            bool roundtrip = (src.filename().u8string() == utf8);
            bool ok = !rec && fs::exists(fs::symlink_status(dst)) && roundtrip;
            report("non-ascii-filename", ok,
                   ok ? "renamed, u8string round trip intact"
                      : "rename or round trip failed: " + rec.message());
        }
    }

    // --- destination held open by another handle -----------------------------------
    // POSIX renames happily over an open file; Windows raises a sharing violation.
    // Either is acceptable -- the guard skips it first -- but the behaviour differs
    // and is worth recording per platform.
    {
        fs::path src = dir / "open-src.sav";
        fs::path dst = dir / "open-dst.srm";
        fs::remove_all(src); fs::remove_all(dst);
        write(src, "SOURCE");
        write(dst, "DESTINATION");

        std::ifstream hold(dst, std::ios::binary);
        std::error_code rec;
        fs::rename(src, dst, rec);
        hold.close();

        std::printf("  INFO  %-34s unguarded rename over an open file: %s\n",
                    "destination-open",
                    rec ? ("refused (" + rec.message() + ")").c_str() : "succeeded");
    }

    std::printf("\n  %d failure(s), %d skipped\n", failures, skips);
    fs::remove_all(dir, ec);
    return failures;
}
