#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Envoltura de linea de comandos para generar el reporte HTML.

Equivalente a `python -m flightreview.report`, pero se puede lanzar directamente:

    python flightreview/scripts/make_report.py data/SIL_sim_servos2.csv -o r.html
"""

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
if _ROOT not in sys.path:
    sys.path.insert(0, _ROOT)

from flightreview.report import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
