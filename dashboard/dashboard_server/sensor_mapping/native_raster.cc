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
    int64_t m=0, remainder=major;
    const int64_t denominator=2*(major>0?major:1), increment=2*minor;
    for (int64_t n=0; n<=major-hit; ++n) {
      const auto xx=sx*(dx>=dy?n:m)-xmin, yy=sy*(dx>=dy?m:n)-ymin;
      bitmap[xx*height+yy]=1;
      remainder+=increment;
      if (remainder>=denominator) {remainder-=denominator;++m;}
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

// Exact x-major bitmap code decoding into one owned Nx2 int64 result. Existing
// ray/occupied precedence, tile updates and packet/admission code are unchanged.
// Complete prevalidation prevents a late invalid code from partially writing.
extern "C" int ubts_coordinate_codes(const int64_t* codes, int64_t count,
    int64_t height, int64_t xmin, int64_t ymin, int64_t* coordinates) {
  if (count<0 || count>4194304 || height<1 || height>4096
      || xmin < -4095 || xmin > 0 || ymin < -4095 || ymin > 0) return 0;
  if (count==0) return 1;
  if (!codes || !coordinates) return 0;
  const auto input=reinterpret_cast<uintptr_t>(codes);
  const auto output=reinterpret_cast<uintptr_t>(coordinates);
  const auto in_bytes=static_cast<uintptr_t>(count)*sizeof(int64_t);
  const auto out_bytes=static_cast<uintptr_t>(count)*2*sizeof(int64_t);
  if (input>UINTPTR_MAX-in_bytes || output>UINTPTR_MAX-out_bytes
      || (input<output+out_bytes && output<input+in_bytes)) return 0;
  const int64_t limit=4096*height<4194304?4096*height:4194304;
  for (int64_t i=0; i<count; ++i)
    if (codes[i]<0 || codes[i]>=limit) return 0;
  for (int64_t i=0; i<count; ++i) {
    const auto row=codes[i]/height;
    coordinates[2*i]=row+xmin;
    coordinates[2*i+1]=codes[i]-row*height+ymin;
  }
  return 1;
}

// Private metadata integration: existing prefix remains byte-identical.
#include <cstdint>
#include <limits>

// Standalone geometry metadata only: no rays, tiles, evidence, clocks or state.
// Every pointer is an owned aligned NumPy allocation in the reviewed wrapper.
namespace {
constexpr int64_t LIMIT = 2147483647;
constexpr int64_t MAX_COUNT = 4194304;
constexpr int64_t MAX_KEYS = 4096;
struct Span { uintptr_t begin, end; };

bool span(const void* pointer, uintptr_t bytes, Span& result) {
  const uintptr_t address = reinterpret_cast<uintptr_t>(pointer);
  if (!pointer || address % alignof(int64_t) || !bytes ||
      address > std::numeric_limits<uintptr_t>::max() - bytes) return false;
  result = {address, address + bytes};
  return true;
}
bool overlaps(Span a, Span b) {
  return a.begin < b.end && b.begin < a.end;
}
int64_t floor_tile(int64_t value) {
  // Origin and relative-coordinate bounds limit |value| to 4294967294.
  return value >= 0 ? value / 64 : -((-value + 63) / 64);
}
}

extern "C" int ubts_coordinate_tile_metadata(
    const int64_t* coordinates, int64_t count, int64_t x0, int64_t y0,
    int64_t* bounds, int64_t* keys, int64_t key_capacity, int64_t* key_count) {
  if (count < 1 || count > MAX_COUNT || x0 < -LIMIT || x0 > LIMIT ||
      y0 < -LIMIT || y0 > LIMIT || key_capacity < 1 ||
      key_capacity > MAX_KEYS) return 0;
  Span buffers[4];
  if (!span(coordinates, static_cast<uintptr_t>(count) * 2 * sizeof(int64_t), buffers[0]) ||
      !span(bounds, 4 * sizeof(int64_t), buffers[1]) ||
      !span(keys, static_cast<uintptr_t>(key_capacity) * 2 * sizeof(int64_t), buffers[2]) ||
      !span(key_count, sizeof(int64_t), buffers[3])) return 0;
  for (int a = 0; a < 4; ++a)
    for (int b = a + 1; b < 4; ++b)
      if (overlaps(buffers[a], buffers[b])) return 0;

  // First pass validates every coordinate before arithmetic/output writes.
  // These are the existing tile updater's bounds, not expanded support.
  int64_t xmin = 0, ymin = 0, xmax = 0, ymax = 0;
  for (int64_t i = 0; i < count; ++i) {
    const int64_t rx = coordinates[2*i], ry = coordinates[2*i+1];
    if (rx < -LIMIT || rx > LIMIT || ry < -LIMIT || ry > LIMIT) return 0;
    const int64_t x = rx + x0, y = ry + y0;
    if (i == 0) { xmin = xmax = x; ymin = ymax = y; }
    else {
      if (x < xmin) xmin = x;
      if (x > xmax) xmax = x;
      if (y < ymin) ymin = y;
      if (y > ymax) ymax = y;
    }
  }
  const int64_t txmin = floor_tile(xmin), tymin = floor_tile(ymin);
  const int64_t nx = floor_tile(xmax) - txmin + 1;
  const int64_t ny = floor_tile(ymax) - tymin + 1;
  if (nx < 1 || ny < 1 || nx > 128 || ny > 128 || nx*ny > MAX_KEYS) return 0;

  // Only a fixed 4096-byte stack bitmap; never an Nx2 absolute/tile array.
  uint8_t present[MAX_KEYS] = {};
  for (int64_t i = 0; i < count; ++i) {
    const int64_t tx = floor_tile(coordinates[2*i] + x0) - txmin;
    const int64_t ty = floor_tile(coordinates[2*i+1] + y0) - tymin;
    // First-pass extrema prove these indices lie in the bounded grid.
    present[tx*ny + ty] = 1;
  }
  int64_t found = 0;
  for (int64_t i = 0; i < nx*ny; ++i) found += present[i];
  if (found > key_capacity) return 0;

  // All validation is complete. No fallible work follows the first write.
  bounds[0] = xmin; bounds[1] = ymin; bounds[2] = xmax; bounds[3] = ymax;
  int64_t output = 0;
  for (int64_t x = 0; x < nx; ++x)
    for (int64_t y = 0; y < ny; ++y)
      if (present[x*ny+y]) {
        keys[2*output] = txmin+x;
        keys[2*output+1] = tymin+y;
        ++output;
      }
  *key_count = found;
  return 1;
}
