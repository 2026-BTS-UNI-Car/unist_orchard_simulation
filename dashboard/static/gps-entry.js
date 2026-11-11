// Hardware loads its offline Korea map; simulation never requests physical basemap tiles.
if (document.documentElement.dataset.runtimeMode === 'simulation') {
  await import('./sim-map.js');
} else {
  await import('./gps-map.js');
}
