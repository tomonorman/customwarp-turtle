from artefacts.other import generate_parameter_output
import yaml
import json
import os


def test_generate_parameter_outptut(tmp_path):
    params = {"turtle/speed": 5}
    file_path = tmp_path / "params.yaml"
    generate_parameter_output(params, file_path)
    with open(file_path) as f:
        out_params = yaml.load(f, Loader=yaml.Loader)
    assert out_params == params

    file_path = tmp_path / "params.json"
    generate_parameter_output(params, file_path)
    with open(file_path) as f:
        ros2_params = json.load(f)
    assert out_params == params
