from datetime import datetime, timezone

import yaml

from artefacts import generate_scenarios, WarpJob


def test_generate_scenarios():
    with open("tests/fixtures/warp.yaml") as f:
        conf = yaml.load(f, Loader=yaml.Loader)
    jobs = conf["jobs"]
    scenarios, first = generate_scenarios(jobs["simple_job"])
    assert len(scenarios) == 2
    scenarios, first = generate_scenarios(jobs["tests"])
    assert first == 0
    assert len(scenarios) == 5
    jobs["simple_job"]["scenarios"]["defaults"] = {"params": {}}
    scenarios, first = generate_scenarios(jobs["simple_job"])
    assert len(scenarios) == 2
    scenarios, first = generate_scenarios(jobs["simple_job"], 0)
    assert len(scenarios) == 1
    scenarios, first = generate_scenarios(jobs["simple_job"], 1)
    assert len(scenarios) == 1
    scenarios, first = generate_scenarios(jobs["tests"], 1)
    assert len(scenarios) == 3
    scenarios, first = generate_scenarios(jobs["tests"], 0)
    assert len(scenarios) == 2


def test_WarpJob():
    job = WarpJob("project", {}, "jobname", {}, dryrun=True)
    assert job.success is False
    assert job.start < datetime.now(timezone.utc).timestamp()
