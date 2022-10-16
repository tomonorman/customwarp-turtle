from artefacts.parameters import iter_grid


def test_iter_grid_empty():
    grid_values = iter_grid({})
    assert list(grid_values) == [{}]


def test_iter_grid():
    earth_gravity = 9.807
    moon_gravity = 1.62

    grid_spec = {
        "gravity": [earth_gravity, moon_gravity],
        "lens_type": ["wide", "standard"],
    }
    grid_values = list(iter_grid(grid_spec))
    assert len(grid_values) == 4
    assert grid_values == [
        {"gravity": earth_gravity, "lens_type": "wide"},
        {"gravity": earth_gravity, "lens_type": "standard"},
        {"gravity": moon_gravity, "lens_type": "wide"},
        {"gravity": moon_gravity, "lens_type": "standard"},
    ]


def test_iter_grid_with_single_values():
    moon_gravity = 1.62

    grid_spec = {"gravity": moon_gravity, "lens_type": "fisheye", "fold": [1, 2]}
    grid_values = list(iter_grid(grid_spec))
    assert len(grid_values) == 2
    assert grid_values == [
        {"gravity": moon_gravity, "lens_type": "fisheye", "fold": 1},
        {"gravity": moon_gravity, "lens_type": "fisheye", "fold": 2},
    ]
