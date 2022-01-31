#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#endif

#include <cmath>
#include <algorithm>
#include <ctime>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include "clipper.h"

using namespace std;
using namespace clipperlib;

//---------------------------------------------------------------------------
// SVGBuilder class
// a very simple class that creates an SVG image file
//---------------------------------------------------------------------------

class SVGBuilder
{
  static string ColorToHtml(unsigned clr)
  {
    stringstream ss;
    ss << '#' << hex << std::setfill('0') << setw(6) << (clr & 0xFFFFFF);
    return ss.str();
  }
  //------------------------------------------------------------------------------

  static float GetAlphaAsFrac(unsigned clr)
  {
    return ((float)(clr >> 24) / 255);
  }
  //------------------------------------------------------------------------------

  class StyleInfo
  {
  public:
  FillRule pft;
  unsigned brushClr;
  unsigned penClr;
  double penWidth;
  bool showCoords;

  StyleInfo()
  {
    pft = FillRule::NonZero;
    brushClr = 0xFFFFFFCC;
    penClr = 0xFF000000;
    penWidth = 0.8;
    showCoords = false;
  }
  };

  class PolyInfo
  {
    public:
      PathsI paths;
    StyleInfo si;

      PolyInfo(PathsI paths, StyleInfo style)
      {
          this->paths = paths;
          this->si = style;
      }
  };

  typedef std::vector<PolyInfo> PolyInfoList;

private:
  PolyInfoList polyInfos;
  static const std::string svg_xml_start[];
  static const std::string poly_end[];

public:
  StyleInfo style;

  void AddPaths(PathsI& poly)
  {
    if (poly.size() == 0) return;
    polyInfos.push_back(PolyInfo(poly, style));
  }

  bool SaveToFile(const string& filename, double scale = 1.0, int margin = 10)
  {
    //calculate the bounding rect ...
    PolyInfoList::size_type i = 0;
    //PathsI::size_type j;
    size_t j;
    while (i < polyInfos.size())
    {
      j = 0;
      while (j < polyInfos[i].paths.size() &&
        polyInfos[i].paths[j].size() == 0) j++;
      if (j < polyInfos[i].paths.size()) break;
      i++;
    }
    if (i == polyInfos.size()) return false;

    RectI rec;
    rec.left = polyInfos[i].paths[j][0].x;
    rec.right = rec.left;
    rec.top = polyInfos[i].paths[j][0].y;
    rec.bottom = rec.top;
    for ( ; i < polyInfos.size(); ++i)
      for (size_t j = 0; j < polyInfos[i].paths.size(); ++j)
        for (size_t k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          PointI ip = polyInfos[i].paths[j][k];
          if (ip.x < rec.left) rec.left = ip.x;
          else if (ip.x > rec.right) rec.right = ip.x;
          if (ip.y < rec.top) rec.top = ip.y;
          else if (ip.y > rec.bottom) rec.bottom = ip.y;
        }

    if (scale == 0) scale = 1.0;
    if (margin < 0) margin = 0;
    rec.left = (int64_t)((double)rec.left * scale);
    rec.top = (int64_t)((double)rec.top * scale);
    rec.right = (int64_t)((double)rec.right * scale);
    rec.bottom = (int64_t)((double)rec.bottom * scale);
    int64_t offsetX = -rec.left + margin;
    int64_t offsetY = -rec.top + margin;

    ofstream file;
    file.open(filename);
    if (!file.is_open()) return false;
    file.setf(ios::fixed);
    file.precision(0);
    file << svg_xml_start[0] <<
      ((rec.right - rec.left) + margin*2) << "px" << svg_xml_start[1] <<
      ((rec.bottom - rec.top) + margin*2) << "px" << svg_xml_start[2] <<
      ((rec.right - rec.left) + margin*2) << " " <<
      ((rec.bottom - rec.top) + margin*2) << svg_xml_start[3];
    setlocale(LC_NUMERIC, "C");
    file.precision(2);

    for (PolyInfoList::size_type i = 0; i < polyInfos.size(); ++i)
  {
      file << " <path d=\"";
    for (size_t j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        file << " M " << ((double)polyInfos[i].paths[j][0].x * scale + offsetX) <<
          " " << ((double)polyInfos[i].paths[j][0].y * scale + offsetY);
        for (size_t k = 1; k < polyInfos[i].paths[j].size(); ++k)
        {
          PointI ip = polyInfos[i].paths[j][k];
          double x = (double)ip.x * scale;
          double y = (double)ip.y * scale;
          file << " L " << (x + offsetX) << " " << (y + offsetY);
        }
        file << " z";
    }
      file << poly_end[0] << ColorToHtml(polyInfos[i].si.brushClr) <<
    poly_end[1] << GetAlphaAsFrac(polyInfos[i].si.brushClr) <<
        poly_end[2] <<
        (polyInfos[i].si.pft == FillRule::EvenOdd ? "evenodd" : "nonzero") <<
        poly_end[3] << ColorToHtml(polyInfos[i].si.penClr) <<
    poly_end[4] << GetAlphaAsFrac(polyInfos[i].si.penClr) <<
        poly_end[5] << polyInfos[i].si.penWidth << poly_end[6];

        if (polyInfos[i].si.showCoords)
        {
      file << "<g font-family=\"Verdana\" font-size=\"11\" fill=\"black\">\n\n";
      for (size_t j = 0; j < polyInfos[i].paths.size(); ++j)
      {
        if (polyInfos[i].paths[j].size() < 3) continue;
        for (size_t k = 0; k < polyInfos[i].paths[j].size(); ++k)
        {
          PointI ip = polyInfos[i].paths[j][k];
          file << "<text x=\"" << (int)(ip.x * scale + offsetX) <<
          "\" y=\"" << (int)(ip.y * scale + offsetY) << "\">" <<
          ip.x << "," << ip.y << "</text>\n";
          file << "\n";
        }
      }
      file << "</g>\n";
        }
    }
    file << "</svg>\n";
    file.close();
    setlocale(LC_NUMERIC, "");
    return true;
  }
}; //SVGBuilder
//------------------------------------------------------------------------------

const std::string SVGBuilder::svg_xml_start [] =
  {"<?xml version=\"1.0\" standalone=\"no\"?>\n"
    "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.0//EN\"\n"
    "\"http://www.w3.org/TR/2001/REC-SVG-20010904/DTD/svg10.dtd\">\n\n"
    "<svg width=\"",
    "\" height=\"",
    "\" viewBox=\"0 0 ",
    "\" version=\"1.0\" xmlns=\"http://www.w3.org/2000/svg\">\n\n"
  };
const std::string SVGBuilder::poly_end [] =
  {"\"\n style=\"fill:",
    "; fill-opacity:",
    "; fill-rule:",
    "; stroke:",
    "; stroke-opacity:",
    "; stroke-width:",
    ";\"/>\n\n"
  };

//------------------------------------------------------------------------------
// Miscellaneous function ...
//------------------------------------------------------------------------------

bool SaveToFile(const string& filename, PathsI &ppg, double scale = 1.0, unsigned decimal_places = 0)
{
  ofstream ofs(filename);
  if (!ofs) return false;

  if (decimal_places > 8) decimal_places = 8;
  ofs << setprecision(decimal_places) << std::fixed;

  PathI pg;
  for (size_t i = 0; i < ppg.size(); ++i)
  {
    for (size_t j = 0; j < ppg[i].size(); ++j)
      ofs << ppg[i][j].x / scale << ", " << ppg[i][j].y / scale << "," << std::endl;
    ofs << std::endl;
  }
  ofs.close();
  return true;
}
//------------------------------------------------------------------------------

bool LoadFromFile(PathsI &ppg, const string& filename, double scale)
{
  //file format assumes: 
  //  1. path coordinates (x,y) are comma separated (+/- spaces) and 
  //  each coordinate is on a separate line
  //  2. each path is separated by one or more blank lines

  ppg.clear();
  ifstream ifs(filename);
  if (!ifs) return false;
  string line;
  PathI pg;
  while (std::getline(ifs, line))
  {
    stringstream ss(line);
    double X = 0.0, Y = 0.0;
    if (!(ss >> X))
    {
      //ie blank lines => flag start of next polygon 
      if (pg.size() > 0) ppg.push_back(pg);
      pg.clear();
      continue;
    }
    char c = ss.peek();  
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces before comma
    if (c == ',') {ss.read(&c, 1); c = ss.peek();} //gobble comma
    while (c == ' ') {ss.read(&c, 1); c = ss.peek();} //gobble spaces after comma
    if (!(ss >> Y)) break; //oops!
    pg.push_back(PointI((int64_t)(X * scale),(int64_t)(Y * scale)));
  }
  if (pg.size() > 0) ppg.push_back(pg);
  ifs.close();
  return true;
}
//------------------------------------------------------------------------------

void MakeRandomPoly(int edgeCount, int width, int height, PathsI & poly)
{
  poly.resize(1);
  poly[0].resize(edgeCount);
  for (int i = 0; i < edgeCount; i++){
    poly[0][i].x = rand() % width;
    poly[0][i].y = rand() % height;
  }
}
//------------------------------------------------------------------------------

bool ASCII_icompare(const char* str1, const char* str2)
{
  //case insensitive compare for ASCII chars only
  while (*str1) 
  {
    if (toupper(*str1) != toupper(*str2)) return false;
    str1++;
    str2++;
  }
  return (!*str2);
}

//------------------------------------------------------------------------------
// Main entry point ...
//------------------------------------------------------------------------------

int main(int argc, char* argv[])
{
  if (argc > 1 &&
    (strcmp(argv[1], "-b") == 0 || strcmp(argv[1], "--benchmark") == 0))
  {
    //do a benchmark test that creates a subject and a clip polygon both with
    //100 vertices randomly placed in a 400 * 400 space. Then perform an
    //intersection operation based on even-odd filling. Repeat all this X times.
    int loop_cnt = 1000;
    char * dummy;
    if (argc > 2) loop_cnt = strtol(argv[2], &dummy, 10);
    if (loop_cnt == 0) loop_cnt = 1000;
    cout << "\nPerforming " << loop_cnt << " random intersection operations ... ";
    srand((int)time(0));
    int error_cnt = 0;
    PathsI subject, clip, solution;
    Clipper clpr;

    time_t time_start = clock();
    for (int i = 0; i < loop_cnt; i++) {
      MakeRandomPoly(100, 400, 400, subject);
      MakeRandomPoly(100, 400, 400, clip);
      clpr.Clear();
      clpr.AddPaths(subject, PathType::Subject, false);
      clpr.AddPaths(clip, PathType::Clip, false);
      if (!clpr.Execute(ClipType::Intersection, FillRule::EvenOdd, solution))
        error_cnt++;
    }
    double time_elapsed = double(clock() - time_start)/CLOCKS_PER_SEC;

    cout << "\nFinished in " << time_elapsed << " secs with ";
    cout << error_cnt << " errors.\n\n";
    //let's save the very last result ...
    SaveToFile("Subject.txt", subject);
    SaveToFile("Clip.txt", clip);
    SaveToFile("Solution.txt", solution);

    //and see the final clipping op as an image too ...
    SVGBuilder svg;
    svg.style.penWidth = 0.8;
    svg.style.pft = FillRule::EvenOdd;
    svg.style.brushClr = 0x1200009C;
    svg.style.penClr = 0xCCD3D3DA;
    svg.AddPaths(subject);
    svg.style.brushClr = 0x129C0000;
    svg.style.penClr = 0xCCFFA07A;
    svg.AddPaths(clip);
    svg.style.brushClr = 0x6080ff9C;
    svg.style.penClr = 0xFF003300;
    svg.style.pft = FillRule::NonZero;
    svg.AddPaths(solution);
    svg.SaveToFile("solution.svg");
    return 0;
  }

  if (argc < 3)
  {
    cout << "\nUsage:\n"
      << "  clipper_console_demo S_FILE C_FILE CT [S_FILL C_FILL] [PRECISION] [SVG_SCALE]\n"
      << "or\n"
      << "  clipper_console_demo --benchmark [LOOP_COUNT]\n\n"
      << "Legend: [optional parameters in square braces]; {comments in curly braces}\n\n"
      << "Parameters:\n"
      << "  S_FILE & C_FILE are the subject and clip input files (see format below)\n"
      << "  CT: cliptype, either INTERSECTION or UNION or DIFFERENCE or XOR\n"
      << "  SUBJECT_FILL & CLIP_FILL: either EVENODD or NONZERO. Default: NONZERO\n"
      << "  PRECISION (in decimal places) for input data. Default = 0\n"
      << "  SVG_SCALE: scale of the output svg image. Default = 1.0\n"
      << "  LOOP_COUNT is the number of random clipping operations. Default = 1000\n\n"
      << "\nFile format for input and output files:\n"
      << "  X, Y[,] {first vertex of first path}\n"
      << "  X, Y[,] {next vertex of first path}\n"
      << "  {etc.}\n"
      << "  X, Y[,] {last vertex of first path}\n"
      << "  {blank line(s) between paths}\n"
      << "  X, Y[,] {first vertex of second path}\n"
      << "  X, Y[,] {next vertex of second path}\n"
      << "  {etc.}\n\n"
      << "Examples:\n"
      << "  clipper_console_demo \"subj.txt\" \"clip.txt\" INTERSECTION EVENODD EVENODD\n"
      << "  clipper_console_demo --benchmark 1000\n";
    return 1;
  }

  int scale_log10 = 0;
  char* dummy;
  if (argc > 6) scale_log10 = strtol(argv[6], &dummy, 10);
  double scale = std::pow(double(10), scale_log10);

  double svg_scale = 1;
  if (argc > 7) svg_scale = strtod(argv[7], &dummy);
  svg_scale /= scale;

  PathsI subject, clip;

  if (!LoadFromFile(subject, argv[1], scale))
  {
    cerr << "\nCan't open the file " << argv[1]
      << " or the file format is invalid.\n";
    return 1;
  }
  if (!LoadFromFile(clip, argv[2], scale))
  {
    cerr << "\nCan't open the file " << argv[2]
      << " or the file format is invalid.\n";
    return 1;
  }

  ClipType clipType = ClipType::Intersection;
  const string sClipType[] = {"INTERSECTION", "UNION", "DIFFERENCE", "XOR"};

  if (argc > 3)
  {
    if (ASCII_icompare(argv[3], "XOR")) clipType = ClipType::Xor;
    else if (ASCII_icompare(argv[3], "UNION")) clipType = ClipType::Union;
    else if (ASCII_icompare(argv[3], "DIFFERENCE")) clipType = ClipType::Difference;
    else clipType = ClipType::Intersection;
  }

  FillRule subj_pft = FillRule::NonZero, clip_pft = FillRule::NonZero;
  if (argc > 5)
  {
    if (ASCII_icompare(argv[4], "EVENODD")) subj_pft = FillRule::EvenOdd;
    if (ASCII_icompare(argv[5], "EVENODD")) clip_pft = FillRule::EvenOdd;
  }

  Clipper c;
  c.AddPaths(subject, PathType::Subject, false);
  c.AddPaths(clip, PathType::Clip, false);
  PathsI solution;

  if (!c.Execute(clipType, subj_pft, solution))
  {
    cout << (sClipType[(int)clipType] + " failed!\n\n");
    return 1;
  }

  //PolyTreeI solutionTree;
  //c.Execute(clipType, subj_pft, solutionTree, solution);

  cout << "\nFinished!\n\n";
  SaveToFile("./tests/solution.txt", solution, scale);

  //let's see the result too ...
  SVGBuilder svg;
  svg.style.penWidth = 0.8;
  svg.style.brushClr = 0x1200009C;
  svg.style.penClr = 0xCCD3D3DA;
  svg.style.pft = subj_pft;
  svg.AddPaths(subject);
  svg.style.brushClr = 0x129C0000;
  svg.style.penClr = 0xCCFFA07A;
  svg.style.pft = clip_pft;
  svg.AddPaths(clip);
  svg.style.brushClr = 0x6080ff9C;
  svg.style.penClr = 0xFF003300;
  svg.style.pft = FillRule::NonZero;
  svg.AddPaths(solution);
  svg.SaveToFile("./tests/solution.svg", svg_scale);

  //finally, show the svg image in the default viewing application
  //system("solution.svg"); 
  return 0;
}
//---------------------------------------------------------------------------
