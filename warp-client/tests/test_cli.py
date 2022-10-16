from artefacts.scripts.warpcli import hello, run, run_remote, add, delete, APIConf
import pytest


@pytest.fixture(scope="class")
def project_with_key(cli_runner):
    project_name = "_pytest-project_"
    cli_runner.invoke(add, [project_name], input="MYAPIKEY\n")
    yield project_name
    cli_runner.invoke(delete, [project_name])


def test_hello(cli_runner):
    project_name = "_pytest-project_"
    result = cli_runner.invoke(hello, [project_name])
    assert result.exit_code == 1
    assert result.output == (
        f"Error: No API KEY set. Please run 'warpcli config add {project_name}'\n"
    )


def test_configure(cli_runner):
    project_name = "_pytest-project_"
    result = cli_runner.invoke(add, [project_name], input="MYAPIKEY\n")
    assert result.output == (
        f"Please enter your API KEY for {project_name}: \n"
        f"API KEY saved for {project_name}\n"
    )
    cli_runner.invoke(delete, [project_name])


def test_run(cli_runner):
    result = cli_runner.invoke(run, ["tests"])
    assert result.exit_code == 1
    assert result.output == "Error: Project config file not found: warp.yaml\n"


def test_run_with_conf_invalid_jobname(cli_runner, project_with_key):
    result = cli_runner.invoke(
        run, ["invalid_job_name", "--config", "tests/fixtures/warp.yaml"]
    )
    assert result.exit_code == 1
    assert result.output == (
        "Connecting to https://app.artefacts.com/api using ApiKey\n"
        f"Starting tests for {project_with_key}\n"
        "Error: Job invalid_job_name not defined\n"
    )


def test_run_with_conf(cli_runner, project_with_key):
    result = cli_runner.invoke(
        run, ["simple_job", "--config", "tests/fixtures/warp.yaml", "--dryrun"]
    )
    assert result.exit_code == 0
    assert result.output == (
        "Connecting to https://app.artefacts.com/api using ApiKey\n"
        f"Starting tests for {project_with_key}\n"
        "Starting scenario 1/2: basic-tests\n"
        "Starting scenario 2/2: other-tests\n"
        "Done\n"
    )


def test_run_with_mode_other(cli_runner, project_with_key):
    result = cli_runner.invoke(
        run,
        ["simple_job", "--config", "tests/fixtures/warp-env-param.yaml", "--dryrun"],
    )
    assert result.exit_code == 0
    assert result.output == (
        "Connecting to https://app.artefacts.com/api using ApiKey\n"
        f"Starting tests for {project_with_key}\n"
        "Starting scenario 1/2: basic-tests\n"
        "Starting scenario 2/2: other-tests\n"
        "Done\n"
    )


def test_run_remote(cli_runner):
    result = cli_runner.invoke(run_remote, ["tests"])
    assert result.exit_code == 1
    assert result.output == "Error: Project config file not found: warp.yaml\n"


def test_run_remote_with_conf_invalid_jobname(cli_runner, project_with_key):
    result = cli_runner.invoke(
        run_remote, ["invalid_job_name", "--config", "tests/fixtures/warp.yaml"]
    )
    assert result.exit_code == 1
    assert result.output == (
        "Connecting to https://app.artefacts.com/api using ApiKey\n"
        "Error: Can't find a job named 'invalid_job_name' in config 'tests/fixtures/warp.yaml'\n"
    )


def test_APIConf(project_with_key):
    conf = APIConf(project_with_key)
    assert conf.headers["Authorization"] == "ApiKey MYAPIKEY"
