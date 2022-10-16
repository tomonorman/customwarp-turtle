import time
import os
import random
import configparser
import logging
import platform
import tarfile
import tempfile
import json
from urllib.parse import urlparse

import yaml
import click
import requests
import traceback

logging.basicConfig()
logging.getLogger().setLevel("INFO")

from artefacts import init_job, generate_scenarios, __version__, AuthenticationError

HOME = os.path.expanduser("~")
CONFIG_DIR = f"{HOME}/.warp"
CONFIG_PATH = f"{CONFIG_DIR}/config"


def get_conf_from_file():
    config = configparser.ConfigParser()
    if not os.path.isfile(CONFIG_PATH):
        os.makedirs(CONFIG_DIR, exist_ok=True)
        config["DEFAULT"] = {}
        with open(CONFIG_PATH, "w") as f:
            config.write(f)
    config.read(CONFIG_PATH)
    return config


class APIConf:
    def __init__(self, project_name):
        config = get_conf_from_file()
        if project_name in config:
            profile = config[project_name]
        else:
            profile = {}
        self.api_url = os.environ.get(
            "WARP_API_URL",
            profile.get(
                "ApiUrl",
                "https://app.artefacts.com/api",
            ),
        )
        self.api_key = os.environ.get("WARP_KEY", profile.get("ApiKey", None))
        if self.api_key is None:
            batch_id = os.environ.get("AWS_BATCH_JOB_ID", None)
            job_id = os.environ.get("ARTEFACTS_JOB_ID", None)
            if batch_id is None or job_id is None:
                raise click.ClickException(
                    f"No API KEY set. Please run 'warpcli config add {project_name}'"
                )
            auth_type = "Internal"
            # Batch id for array jobs contains array index
            batch_id = batch_id.split(":")[0]
            self.headers = {"Authorization": f"{auth_type} {job_id}:{batch_id}"}
        else:
            auth_type = "ApiKey"
            self.headers = {"Authorization": f"{auth_type} {self.api_key}"}
        self.headers[
            "User-Agent"
        ] = f"ArtefactsClient/{__version__} ({platform.platform()}/{platform.python_version()})"
        click.echo(f"Connecting to {self.api_url} using {auth_type}")
        click.echo(f"HEADERS {self.headers}")


def read_config(filename):
    with open(filename, "r") as f:
        config = yaml.load(f, Loader=yaml.Loader)
    return config


@click.group()
def config():
    return


@config.command()
@click.argument("project_name")
def add(project_name):
    """
    Set configuration for PROJECT_NAME
    """
    api_key = click.prompt(
        f"Please enter your API KEY for {project_name}", type=str, hide_input=True
    )
    config = get_conf_from_file()
    config[project_name] = {"ApiKey": api_key}
    with open(CONFIG_PATH, "w") as f:
        config.write(f)
    click.echo(f"API KEY saved for {project_name}")
    return


@config.command()
@click.argument("project_name")
def delete(project_name):
    """
    Delete configuration for PROJECT_NAME
    """
    config = get_conf_from_file()
    config.remove_section(project_name)
    with open(CONFIG_PATH, "w") as f:
        config.write(f)
    click.echo(f"{project_name} config removed")


@click.command()
@click.argument("project_name")
def hello(project_name):
    """Show message to confirm credentials allow access to PROJECT_NAME"""
    api_conf = APIConf(project_name)
    response = requests.get(
        f"{api_conf.api_url}/{project_name}/info",
        headers=api_conf.headers,
    )
    if response.status_code == 200:
        result = response.json()
        click.echo(
            "Hello " + click.style(f"{result['name']}@{result['framework']}", fg="blue")
        )
    else:
        result = response.json()
        raise click.ClickException(f"Error getting project info: {result['message']}")


@click.command()
@click.option("--config", default="warp.yaml", help="Warp config file.")
@click.option(
    "--dryrun",
    is_flag=True,
    default=False,
    help="Dryrun: no tracking or test execution",
)
@click.argument("jobname")
def run(config, jobname, dryrun):
    """Run JOBNAME locally"""
    print("THIS IS FROM A CUSTOM CLI")
    try:
        warpconfig = read_config(config)
    except FileNotFoundError:
        raise click.ClickException(f"Project config file not found: {config}")

    project_id = warpconfig["project"]
    api_conf = APIConf(project_id)
    click.echo(f"Starting tests for {project_id}")
    if jobname not in warpconfig["jobs"]:
        raise click.ClickException(f"Job {jobname} not defined")
    jobconf = warpconfig["jobs"][jobname]
    job_type = jobconf.get("type", "test")
    if job_type not in ["test"]:
        click.echo(f"Job type not supported: f{job_type}")
        return
    supported_frameworks = ["ros2:0", "other"]
    framework = jobconf["runtime"]["framework"]
    if framework not in supported_frameworks:
        click.echo(
            f"Job framework not supported: {framework}. Supprted frameworks: {supported_frameworks}"
        )
        return
    batch_index = os.environ.get("AWS_BATCH_JOB_ARRAY_INDEX", None)
    if batch_index is not None:
        batch_index = int(batch_index)
        click.echo(f"AWS BATCH ARRAY DETECTED, batch_index={batch_index}")
    scenarios, first = generate_scenarios(jobconf, batch_index)
    print(f"project_id'{project_id}', api_conf'{api_conf}', jobname'{jobname}', jobconf'{jobconf}', dryrun'{dryrun}', first'{first}',")
    try:
        warpjob = init_job(project_id, api_conf, jobname, jobconf, dryrun, first)
    except AuthenticationError:
        logging.error(traceback.format_exc())
        raise click.ClickException(
            "Unable to authenticate, check your Project Name and API Key"
        )

    job_success = True
    for scenario_n, scenario in enumerate(scenarios):
        click.echo(
            f"Starting scenario {scenario_n+1}/{len(scenarios)}: {scenario['name']}"
        )
        try:
            run = warpjob.new_run(scenario)
        except AuthenticationError:
            logging.error(traceback.format_exc())
            raise click.ClickException(
                "Unable to authenticate, check your Project Name and API Key"
            )
        if framework == "ros2:0":
            from artefacts.ros2 import run_ros2_tests

            if "ros2_testfile" not in run.params:
                raise click.ClickException(
                    "Test launch file not specified for ros2 project"
                )
            if dryrun:
                results, success = {}, True
            else:
                results, success = run_ros2_tests(run)
            if not success:
                job_success = False
        else:
            from artefacts.other import run_other_tests

            if "run" not in run.params:
                raise click.ClickException("run command not specified for scenario")
            if dryrun:
                results, success = {}, True
            else:
                results, success = run_other_tests(run)
            if not success:
                job_success = False
            if type(run.params.get("metrics", [])) == str:
                run.log_metrics()

        run.stop()
    warpjob.log_tests_result(job_success)
    click.echo("Done")
    time.sleep(random.random() * 1)

    warpjob.stop()


@click.command()
@click.option("--config", default="warp.yaml", help="Warp config file.")
@click.option(
    "--description",
    default="Test local source",
    help="Optional description for this run",
)
@click.argument("jobname")
def run_remote(config, description, jobname):
    """
    Run JOBNAME in the cloud

    This command requires to have a linked GitHub repository
    """
    try:
        warpconfig = read_config(config)
    except FileNotFoundError:
        raise click.ClickException(f"Project config file not found: {config}")

    project_id = warpconfig["project"]
    api_conf = APIConf(project_id)
    project_folder = os.path.dirname(os.path.abspath(config))
    dashboard_url = urlparse(api_conf.api_url)
    dashboard_url = f"{dashboard_url.scheme}://{dashboard_url.netloc}/{project_id}"

    try:
        warpconfig["jobs"][jobname]
    except KeyError:
        raise click.ClickException(
            f"Can't find a job named '{jobname}' in config '{config}'"
        )

    # Keep only the selected job in the config
    run_config = warpconfig.copy()
    run_config["jobs"] = {jobname: warpconfig["jobs"][jobname]}
    if "on" in run_config:
        del run_config["on"]

    click.echo(f"Packaging source...")

    with tempfile.NamedTemporaryFile(
        prefix=project_id, suffix=".tgz", delete=True
    ) as temp_file:
        with tarfile.open(fileobj=temp_file, mode="w:gz") as tar_file:
            for root, dirs, files in os.walk(project_folder):
                for file in files:
                    absolute_path = os.path.join(root, file)
                    relative_path = os.path.relpath(absolute_path, project_folder)
                    # ignore .git folder
                    if relative_path.startswith(".git/"):
                        continue
                    # TODO ignore files with .gitignore
                    tar_file.add(absolute_path, arcname=relative_path, recursive=False)

        temp_file.flush()
        temp_file.seek(0)

        # Request signed upload URLs
        upload_urls_response = requests.put(
            f"{api_conf.api_url}/{project_id}/upload_source",
            headers=api_conf.headers,
        )

        if not upload_urls_response.ok:
            result = upload_urls_response.json()
            if (
                upload_urls_response.status_code == 403
                and result["message"] == "Not allowed"
            ):
                raise click.ClickException(
                    f"Missing access! Please make sure your API key is added at {dashboard_url}/settings"
                )

            if (
                upload_urls_response.status_code == 401
                and result["message"] == "no linked repository"
            ):
                raise click.ClickException(
                    f"Missing linked GitHub repository. Please link a GitHub repository at {dashboard_url}/settings"
                )

            raise click.ClickException(
                f"Error getting project info: {result['message']}"
            )

        upload_urls = upload_urls_response.json()["upload_urls"]

        # Mock the necessary parts of the GitHub event
        integration_payload = {
            "head_commit": {
                # shown on the dashboard in the job details
                "message": description,
                "url": "",
            },
            "repository": {
                # used by the container-builder for creating the ecr repo name
                "full_name": project_id,
            },
            # special key to distinguish the valid GitHub payload from these fabricated ones
            "ref": "local-source",
            "after": "",
        }

        uploads = [
            ("archive.tgz", temp_file),
            ("warp.yaml", ("warp.yaml", yaml.dump(run_config))),
            (
                "integration_payload.json",
                ("integration_payload.json", json.dumps(integration_payload)),
            ),
        ]

        with click.progressbar(
            uploads, label="Uploading", item_show_func=lambda x: x and x[0]
        ) as bar:
            for (filename, file) in bar:
                response = requests.post(
                    upload_urls[filename]["url"],
                    data=upload_urls[filename]["fields"],
                    files={"file": file},
                )
                if not response.ok:
                    raise click.ClickException(
                        f"Failed to upload {filename}: {response.text}"
                    )

        click.echo(
            f"Uploading complete! The new job will show up shortly at {dashboard_url}/tests"
        )


@click.group()
@click.version_option(version=__version__)
def warpcli():
    """A command line tool to interface with WARP"""
    compute_env = os.getenv("AWS_BATCH_CE_NAME", "")
    if compute_env != "":
        click.echo(f"running version {__version__}")
        if "development" in compute_env and os.getenv("WARP_API_URL", None) is None:
            os.environ["WARP_API_URL"] = "https://ui.artefacts.com/api"


warpcli.add_command(config)
warpcli.add_command(hello)
warpcli.add_command(run)
warpcli.add_command(run_remote)


if __name__ == "__main__":
    warpcli()
