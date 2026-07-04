"""ODrive single-axis CAN backend for the foc_ui upper-computer.

Provides CAN communication with the trimmed ODrive v3.6 production firmware
(single Axis0, 1 Mbps, node_id=0) via python-can.

Modules:
    protocol   - CAN frame IDs, enums, encode/decode functions
    transport  - python-can Bus wrapper with reader thread
    state      - Axis state cache + telemetry synthesizer
    service    - Orchestration: polling, commands, safety monitor
    ws_hub     - CAN-aware WebSocket hub
    recorder   - CAN CSV recorder
    main       - FastAPI routes + WebSocket endpoint
"""

__version__ = '0.1.0'
