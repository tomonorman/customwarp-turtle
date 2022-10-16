# WARP Client

Python client and CLI for WARP


## CLI

To install:
```
pip install --editable .
```

Check configuration: after creating `project-name` from the web UI and getting an API key, try:

```
warpcli config add [project-name]
```

and enter you `API_KEY` for the project when prompted.

You can then do 

```
warpcli hello [project-name]
```

alternatively, you can specify you API KEY via environment variable

```
WARP_KEY=[your-key] warpcli hello [project-name]
```

To run a job locally, for example the turtlesim example (need ros2 installed).
First edit `warp.yaml` to change the project name, then:

```
cd examples/turtlesim
warpcli run basic_tests
```

## Configuration file syntax

See [the configuration syntax documentation](docs/configuration_syntax.md)

## Development

For the client/CLI dev environment, additionally install:

```
pip install -r requirements.txt
```

You can run the tests with:

```
pytest
```

If you need to change the API url, you can:

* Edit `~/.warp/config`, and add `apiurl = http://localhost:5000/api` in the `[DEFAULT]` section
* Using an environment variable, `WARP_API_URL=http://localhost:5000/api warpcli hello [project-name]`

### _Note_ when using Docker to Run a Job Locally

When using the client/cli dev environment on your machine, but building and running a job through Docker, e.g 
```
docker run --env WARP_KEY=<ApiKey> --env ARTEFACTS_JOB_NAME=basic_tests --env WARP_API_URL=<yourlocalhostUrl>  <tag>
```
(such as the Dockerfile in the [dolly-demo](https://github.com/art-e-fact/dolly-demo/blob/main/Dockerfile) repo)

You need to point the WARP_API_URL back to your host machine which is `host.docker.internal` i.e `WARP_API_URL=http://host.docker.internal:5000`

## Release management

the github action CI/CD is automatically generating PEP503/PEP440 compatible release on a website enabled s3 bucket.

They can be seen at [this url](http://public-artefacts-releases.s3-website-ap-northeast-1.amazonaws.com/pep503/artefacts-client/]

Usage with pip is

```
pip install artefacts-client --extra-index-url https://d5cw4z7oemmfd.cloudfront.net/pep503/
```

Dev versions could be made available and installed by adding the `--pre` flag to the command. However to limit installation of dev versions, the dev versions are for now only published to the a non pep503 location but can be installed with:

```
pip install http://public-artefacts-releases.s3-website-ap-northeast-1.amazonaws.com/artefacts-client-dev.zip
```
