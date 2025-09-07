from opendbc.car.structs import CarParams
from opendbc.car.byd.values import CAR

Ecu = CarParams.Ecu

FINGERPRINTS = {
  CAR.BYD_HAN_EV_23: [{
    140: 8, 213: 8, 287: 5, 289: 8, 291: 8, 301: 8, 307: 8, 337: 8, 496: 8, 536: 8, 544: 8, 546: 8, 547: 8, 575: 8, 576: 8
  }],
}

#Todo: Get a byd VDS to see how fw could be queried. Currently added just for preventing ruffs error.
FW_VERSIONS: dict[str, dict[tuple, list[bytes]]] = {
}
