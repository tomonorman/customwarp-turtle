import subprocess
import json
import yaml
import os
from pathlib import Path


def generate_parameter_output(params: dict, output_filename: os.PathLike):
    """Store `params` in `output_filename` in either json or yaml"""
    with open(output_filename, "w+") as f:
        if output_filename.suffix == ".json":
            json.dump(params, f)
        elif output_filename.suffix == ".yaml":
            yaml.dump(params, f)
        else:
            raise NotImplementedError("Please specify a json or yaml file")


def run_other_tests(run):
    env = None
    if "params_output" in run.params:
        if "params" in run.params:
            generate_parameter_output(
                run.params["params"], Path(run.params["params_output"])
            )
    else:
        env = run.params.get("params", None)
    command = run.params["run"]
    if env is not None:
        full_env = {**os.environ, **env}
        subprocess.run(
            command, shell=True, env={k: str(v) for k, v in full_env.items()}
        )
    else:
        subprocess.run(command, shell=True)
    results = []
    success = True
    run.log_artifacts(run.output_path)
    run.log_tests_results(results, success)
    return results, success
