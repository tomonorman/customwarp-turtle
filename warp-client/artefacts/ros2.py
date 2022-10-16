import yaml
import subprocess
from glob import glob

from junitparser import JUnitXml, Attr, Element


class FailureElement(Element):
    _tag = "failure"
    message = Attr()


def parse_tests_results(file):
    xml = JUnitXml.fromfile(file)
    results = []
    success = True
    for suite in xml:
        # handle suites
        suite_results = {
            "suite": suite.name,
            "errors": suite.errors,
            "failures": suite.failures,
            "tests": suite.tests,
        }
        details = []
        for case in suite:
            case_details = {
                "name": case.name,
            }
            try:
                case_details["failure_message"] = case.child(FailureElement).message
                case_details["result"] = "failure"
                success = False
            except AttributeError:
                case_details["result"] = "success"
            details.append(case_details)
        suite_results["details"] = details
        results.append(suite_results)
    return results, success


def generate_parameter_output(params: dict, output_filename: str):
    """
    Store `params` in `output_filename` and convert to ros2 param file nested format,
    to be used by the launch file
    """
    content = {}
    for k, v in params.items():
        node, pname = k.split("/")
        if node not in content:
            content[node] = {"ros__parameters": {}}
        content[node]["ros__parameters"][pname] = v
    with open(output_filename, "w+") as f:
        yaml.dump(content, f)
    return


def run_ros2_tests(run):
    scenario = run.params
    launch_file = scenario["ros2_testfile"]
    # TODO: HOW TO ADD  NODE to launch
    # TODO: set params from conf
    # TODO: get params to log
    # TODO: where is the rosbag
    if "params" in run.params:
        generate_parameter_output(run.params["params"], run.params["params_output"])
    preexisting_rosbags = glob("rosbag2*")
    test_result_file_path = f"{run.output_path}/tests_junit.xml"
    subprocess.run(["launch_test", launch_file, "--junit-xml", test_result_file_path])
    results, success = parse_tests_results(test_result_file_path)
    run.log_artifacts(run.output_path)
    for output in scenario.get("output_dirs", []):
        run.log_artifacts(output["path"], output["name"])
    # check if any rosbag was created

    rosbags = glob("rosbag2*")
    new_rosbags = set(rosbags).difference(set(preexisting_rosbags))
    from artefacts.bagparser import BagFileParser

    if len(new_rosbags) > 0:
        rosbag_path = new_rosbags.pop()
        run.log_artifacts(rosbag_path, "rosbag")
        if "metrics" in run.params:
            print(rosbag_path)
            db_file = glob(f"{rosbag_path}/*.db3")[
                0
            ]  # TODO should go inside BagFileParser?
            bag = BagFileParser(db_file)
            for metric in run.params["metrics"]:
                last_value = bag.get_last_message(metric)[1].data
                print(last_value)
                run.log_metric(metric, last_value)

    run.log_tests_results(results, success)
    return results, success
