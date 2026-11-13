// Exact bounded Bresenham ray union. Coordinates use the Python x-major bitmap.
// No global state, allocation, sensor interpretation or localization logic.
#include <cstdint>
#include <cstdlib>
#include <cstring>

extern "C" int ubts_raster(const int64_t* rays, int64_t count, int64_t xmin,
    int64_t ymin, int64_t width, int64_t height, int64_t margin, uint8_t* bitmap) {
  if (!rays || !bitmap || count < 1 || count > 20000 || width < 1 || height < 1
      || width > 4096 || height > 4096 || width*height > 4194304
      || xmin < -4095 || xmin > 0 || ymin < -4095 || ymin > 0
      || margin < 0 || margin > 8) return 0;
  for (int64_t i=0; i<count; ++i) {
    const auto x=rays[3*i], y=rays[3*i+1], hit=rays[3*i+2];
    if (x < xmin || x >= xmin+width || y < ymin || y >= ymin+height
        || std::abs(x)>4095 || std::abs(y)>4095 || (hit!=0 && hit!=1)) return 0;
  }
  std::memset(bitmap, 0, static_cast<size_t>(width*height));
  for (int64_t i=0; i<count; ++i) {
    const auto x=rays[3*i], y=rays[3*i+1], hit=rays[3*i+2];
    const int64_t dx=std::abs(x), dy=std::abs(y), major=dx>dy?dx:dy,
      minor=dx>dy?dy:dx, sx=(x>0)-(x<0), sy=(y>0)-(y<0);
    for (int64_t n=0; n<=major-hit; ++n) {
      const auto m=(2*n*minor+major)/(2*(major>0?major:1));
      const auto xx=sx*(dx>=dy?n:m)-xmin, yy=sy*(dx>=dy?m:n)-ymin;
      bitmap[xx*height+yy]=1;
    }
  }
  // Occupied wins over every free ray, including neighboring hit margins.
  for (int64_t i=0; i<count; ++i) if (rays[3*i+2]) {
    const auto x=rays[3*i]-xmin, y=rays[3*i+1]-ymin;
    for (int64_t ax=-margin; ax<=margin; ++ax)
      for (int64_t ay=-margin; ay<=margin; ++ay) {
        const auto xx=x+ax, yy=y+ay;
        if (xx>=0 && xx<width && yy>=0 && yy<height) bitmap[xx*height+yy]&=2;
      }
    bitmap[x*height+y]=2;
  }
  return 1;
}

// The caller supplies owned NumPy tile pointers and unique raster coordinates.
// Validate every lookup before touching any tile, then apply exactly one update
// per cell. A compact pointer table avoids sorting/grouping every free cell.
extern "C" int64_t ubts_update_tiles(const int64_t* coordinates, int64_t count,
    int64_t x0, int64_t y0, int64_t delta, int64_t conservative,
    const uintptr_t* table, int64_t txmin, int64_t tymin, int64_t nx, int64_t ny) {
  if (!coordinates || !table || count<1 || count>4194304 || nx<1 || ny<1
      || nx>128 || ny>128 || nx*ny>4096 || delta < -127 || delta > 127
      || x0 < -2147483647 || x0 > 2147483647
      || y0 < -2147483647 || y0 > 2147483647) return -1;
  auto tile_index = [](int64_t value) { return value>=0?value/64:-((-value+63)/64); };
  for (int64_t i=0; i<count; ++i) {
    auto rx=coordinates[2*i], ry=coordinates[2*i+1];
    if (rx < -2147483647 || rx > 2147483647 || ry < -2147483647 || ry > 2147483647) return -1;
    auto x=rx+x0,y=ry+y0,tx=tile_index(x)-txmin,ty=tile_index(y)-tymin;
    if (tx<0 || tx>=nx || ty<0 || ty>=ny || !table[tx*ny+ty]) return -1;
  }
  int64_t fresh=0;
  for (int64_t i=0; i<count; ++i) {
    auto x=coordinates[2*i]+x0,y=coordinates[2*i+1]+y0;
    auto tx=tile_index(x),ty=tile_index(y);
    auto tile=reinterpret_cast<int8_t*>(table[(tx-txmin)*ny+ty-tymin]);
    auto& cell=tile[(y-ty*64)*64+x-tx*64];
    int64_t value=cell;
    if (value==127) {value=0;++fresh;}
    value+=(conservative && delta<0 && value>=9)?-1:delta;
    cell=static_cast<int8_t>(value<-40?-40:value>60?60:value);
  }
  return fresh;
}
