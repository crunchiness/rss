X_MAX = 320.
Y_MAX = 525.

# edge r to r+s, tuples in the (r, s) format, not (r, r+s)
ARENA_WALLS = [
    ([0.0, 0.0], [0.0, Y_MAX]),
    ([0.0, 0.0], [X_MAX, 0.0]),
    ([0.0, Y_MAX], [X_MAX, 0.0]),
    ([X_MAX, 0.0], [0.0, Y_MAX]),

    ([143.0, 0.0], [0.0, 93.0]),
    ([143.0, 93.0], [32.0, 0.0]),

    ([0.0, 132.0], [74.0, 0.0]),

    ([0.0, 294.0], [93.0, 0.0]),
    ([93.0, 294.0], [0.0, 46.0]),
    ([93.0, 340.0], [34.0, 0.0]),

    ([X_MAX, 231.0], [-93.0, 0.0]),
    ([X_MAX - 93.0, 231.0], [0.0, -46.0]),
    ([X_MAX - 93.0, 231.0 - 46.0], [-34.0, 0.0]),

    ([X_MAX, Y_MAX - 132.0], [-74.0, 0.0]),

    ([177.0, Y_MAX], [0.0, -93.0]),
    ([177.0, Y_MAX - 93.0], [-32.0, 0.0]),

    ([X_MAX - 70.0, 91.0], [-23.0, 0.0]),
    ([X_MAX - 70.0, 91.0], [0.0, 23.0]),
    ([X_MAX - 93.0, 114.0], [23.0, 0.0]),
    ([X_MAX - 93.0, 114.0], [0.0, -23.0]),

    ([70.0, Y_MAX - 91.0], [23.0, 0.0]),
    ([70.0, Y_MAX - 91.0], [0.0, -23.0]),
    ([93.0, Y_MAX - 114.0], [-23.0, 0.0]),
    ([93.0, Y_MAX - 114.0], [0.0, 23.0]),

    ([X_MAX - 143.0, 234.0+15.0], [-38.0, 0.0]),
    ([X_MAX - 143.0, 234.0+15.0], [0.0, 31.0]),
    ([X_MAX - 143.0 - 38.0, 234.0 + 31.0+15.0], [38.0, 0.0]),
    ([X_MAX - 143.0 - 38.0, 234.0 + 31.0+15.0], [0.0, -31.0])
]

NODES = {
    'A1': {'id': 'A1', 'room': 'F', 'y': 50, 'x': 70, 'ambiguous': False, 'connects': ['B1']},
    'A2': {'id': 'A2', 'room': 'A', 'y': 475, 'x': 250, 'ambiguous': False, 'connects': ['B2']},
    'B1': {'id': 'B1', 'room': 'F', 'y': 75, 'x': 100, 'ambiguous': True, 'connects': ['C1', 'A1']},
    'B2': {'id': 'B2', 'room': 'A', 'y': 450, 'x': 220, 'ambiguous': True, 'connects': ['C2', 'A2']},
    'C1': {'id': 'C1', 'room': 'D', 'y': 175, 'x': 120, 'ambiguous': True, 'connects': ['B1', 'D1', 'E1', 'H1', 'I1']},
    'C2': {'id': 'C2', 'room': 'C', 'y': 350, 'x': 200, 'ambiguous': True, 'connects': ['B2', 'D2', 'E2', 'H2', 'I2']},
    'D1': {'id': 'D1', 'room': 'D', 'y': 200, 'x': 50, 'ambiguous': False, 'connects': ['C1', 'E1', 'I1']},
    'D2': {'id': 'D2', 'room': 'C', 'y': 325, 'x': 270, 'ambiguous': False, 'connects': ['C2', 'E2', 'I2']},
    'E1': {'id': 'E1', 'room': 'D', 'y': 215, 'x': 160, 'ambiguous': True, 'connects': ['D1', 'C1', 'L2']},
    'E2': {'id': 'E2', 'room': 'C', 'y': 310, 'x': 160, 'ambiguous': True, 'connects': ['D2', 'C2', 'L1']},
    'F1': {'id': 'F1', 'room': 'E', 'y': 155, 'x': 280, 'ambiguous': False, 'connects': ['G1', 'K1']},
    'F2': {'id': 'F2', 'room': 'B', 'y': 370, 'x': 40, 'ambiguous': False, 'connects': ['G2', 'K2']},
    'G1': {'id': 'G1', 'room': 'E', 'y': 155, 'x': 250, 'ambiguous': False, 'connects': ['F1', 'H1']},
    'G2': {'id': 'G2', 'room': 'B', 'y': 370, 'x': 70, 'ambiguous': False, 'connects': ['F2', 'H2']},
    'H1': {'id': 'H1', 'room': 'E', 'y': 155, 'x': 200, 'ambiguous': True, 'connects': ['C1', 'G1', 'J1']},
    'H2': {'id': 'H2', 'room': 'B', 'y': 370, 'x': 120, 'ambiguous': True, 'connects': ['C2', 'G2', 'J2']},
    'I1': {'id': 'I1', 'room': 'D', 'y': 275, 'x': 115, 'ambiguous': True, 'connects': ['C1', 'D1', 'L1']},
    'I2': {'id': 'I2', 'room': 'C', 'y': 250, 'x': 205, 'ambiguous': True, 'connects': ['C2', 'D2', 'L2']},
    'J1': {'id': 'J1', 'room': 'E', 'y': 50, 'x': 200, 'ambiguous': False, 'connects': ['K1', 'H1']},
    'J2': {'id': 'J2', 'room': 'B', 'y': 475, 'x': 120, 'ambiguous': False, 'connects': ['K2', 'H2']},
    'K1': {'id': 'K1', 'room': 'E', 'y': 50, 'x': 280, 'ambiguous': False, 'connects': ['J1', 'F1']},
    'K2': {'id': 'K2', 'room': 'B', 'y': 475, 'x': 40, 'ambiguous': False, 'connects': ['J2', 'F2']},
    'L1': {'id': 'L1', 'room': 'C2', 'y': 310, 'x': 120, 'ambiguous': True, 'connects': ['I1', 'E2']},
    'L2': {'id': 'L2', 'room': 'C1', 'y': 215, 'x': 200, 'ambiguous': True, 'connects': ['I2', 'E1']}
}
