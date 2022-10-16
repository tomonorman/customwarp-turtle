import json
import glob
from datetime import datetime, timezone
import logging
import os

import requests

from .parameters import iter_grid

logging.basicConfig(level=logging.INFO)

__version__ = "0.2.16"


class AuthenticationError(Exception):
    """Raised when artefacts authentication failed"""

    pass


class WarpJob:
    def __init__(
        self, project_id, api_conf, jobname, jobconf, dryrun=False, run_offset=0
    ):
        self.project_id = project_id
        self.job_id = os.environ.get("ARTEFACTS_JOB_ID", None)
        self.api_conf = api_conf
        self.start = datetime.now(timezone.utc).timestamp()
        self.uploads = {}
        self.jobname = jobname
        self.params = jobconf
        self.success = False
        self.n_runs = run_offset
        self.dryrun = dryrun

        if dryrun:
            self.job_id = "dryrun"
        if self.job_id is None:
            data = {
                "start": int(self.start),
                "status": "in progress",
                "params": json.dumps(self.params),
                "project": self.project_id,
                "jobname": self.jobname,
                "timeout": self.params.get("timeout", 5) * 60,
            }
            response = requests.post(
                f"{api_conf.api_url}/{self.project_id}/job",
                json=data,
                headers=api_conf.headers,
            )
            if response.status_code != 200:
                if response.status_code == 403:
                    msg = response.json()["message"]
                    logging.warning(msg)
                    raise AuthenticationError(msg)
                logging.warning(f"Error on job creation: {response.status_code}")
                logging.warning(response.text)
                raise AuthenticationError(str(response.status_code))
            self.job_id = response.json()["job_id"]
        self.output_path = self.params.get("output_path", f"/tmp/{self.job_id}")
        os.makedirs(self.output_path, exist_ok=True)
        return

    def log_tests_result(self, success):
        self.success = success

    def stop(self):
        end = datetime.now(timezone.utc).timestamp()
        if self.dryrun:
            return
        # Log metadata
        data = {
            "start": int(self.start),
            "end": int(end),
            "duration": int(end - self.start),
            "project": self.project_id,
            "jobname": self.jobname,
            "success": self.success,
            "params": json.dumps(self.params),
            "status": "finished",
        }
        response = requests.put(
            f"{self.api_conf.api_url}/{self.project_id}/job/{self.job_id}",
            json=data,
            headers=self.api_conf.headers,
        )

        return

    def new_run(self, scenario):
        run = WarpRun(self, scenario, self.n_runs)
        self.n_runs += 1
        return run


class WarpRun:
    def __init__(self, job, scenario, run_n):
        self.job = job
        self.start = datetime.now(timezone.utc).timestamp()
        self.uploads = {}
        self.params = scenario
        self.metrics = {}
        self.run_n = run_n
        self.output_path = self.params.get(
            "output_path", f"{self.job.output_path}/{self.run_n}"
        )
        os.makedirs(self.output_path, exist_ok=True)
        data = {
            "job_id": job.job_id,
            "run_n": self.run_n,
            "start": int(self.start),
            "tests": [],
            "params": json.dumps(self.params),
        }

        if self.job.dryrun:
            return
        query_url = (
            f"{self.job.api_conf.api_url}/{self.job.project_id}/job/{job.job_id}/run"
        )
        response = requests.post(
            query_url,
            json=data,
            headers=self.job.api_conf.headers,
        )
        if response.status_code != 200:
            if response.status_code == 403:
                msg = response.json()["message"]
                logging.warning(msg)
                raise AuthenticationError(msg)
            logging.warning(f"Error on scenario creation: {response.status_code}")
            logging.warning(response.text)
            raise AuthenticationError(str(response.status_code))
        return

    def log_params(self, params):
        self.params = params

    def log_metric(self, name, value):
        self.metrics[name] = value

    def log_metrics(self):
        metrics = self.params.get("metrics", None)
        if type(metrics) == str:
            with open(f"{self.output_path}/{metrics}") as f:
                metric_values = json.load(f)
            for k, v in metric_values.items():
                self.log_metric(k, v)

    def log_tests_results(self, test_results, success):
        self.test_results = test_results
        self.success = success

    def log_artifacts(self, output_path, prefix=None):
        files = [f for f in glob.glob(f"{output_path}/**", recursive=True) if "." in f]
        print(files)
        if prefix is not None:
            self.uploads.update(
                {f"{prefix}/{f.split(f'{output_path}/')[-1]}": f for f in files}
            )
        else:
            self.uploads.update({f.split(f"{output_path}/")[-1]: f for f in files})

    def stop(self):
        end = datetime.now(timezone.utc).timestamp()
        if self.job.dryrun:
            return
        # Log metadata
        data = {
            "job_id": self.job.job_id,
            "run_n": self.run_n,
            "start": int(self.start),
            "params": json.dumps(self.params),
            "end": int(end),
            "duration": int(end - self.start),
            "tests": self.test_results,
            "success": self.success,
            "uploads": self.uploads,
            "metrics": self.metrics,
        }

        response = requests.put(
            f"{self.job.api_conf.api_url}/{self.job.project_id}/job/{self.job.job_id}/run/{self.run_n}",
            json=data,
            headers=self.job.api_conf.headers,
        )
        # use s3 presigned urls to upload the artifacts
        upload_urls = response.json()["upload_urls"]
        for key, file_name in self.uploads.items():
            files = {"file": open(file_name, "rb")}

            upload_info = upload_urls[key]
            r = requests.post(
                upload_info["url"],
                data=upload_info["fields"],
                files=files,
            )


def init_job(
    project_id: str,
    api_token: str,
    jobname: str,
    jobconf: dict,
    dryrun: bool = False,
    run_offset=0,
):
    return WarpJob(project_id, api_token, jobname, jobconf, dryrun, run_offset)


def generate_scenarios(jobconf, scenario_n=None):
    """Create each scenario conf by:
    1. selecting only named scenario specified by scenario_n (for parallel processing)
    2. merging default values to each scenario
    3. generating parameter grids
    """
    scenarios = sorted(jobconf["scenarios"]["settings"], key=lambda x: x["name"])
    defaults = jobconf["scenarios"].get("defaults", {})
    first_scenario = 0
    last_scenario = None
    output_scenarios = []
    for n, scenario_conf in enumerate(scenarios):
        if scenario_n is not None:
            if n == scenario_n:
                first_scenario = len(output_scenarios)
            if n == scenario_n + 1:
                last_scenario = len(output_scenarios)
        scenario = defaults.copy()
        scenario.update(scenario_conf)
        if "params" in scenario:
            grid_values = iter_grid(scenario["params"])
            for settings in grid_values:
                grid_scenario = scenario.copy()
                grid_scenario["params"] = settings
                # TODO better namings
                output_scenarios.append(grid_scenario)

        else:
            output_scenarios.append(scenario)
    return output_scenarios[first_scenario:last_scenario], first_scenario
