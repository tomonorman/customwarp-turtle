.PHONY: build lint format test npm_ci

# ====== Check ======

install:
	pip install -e .
	pip install -r requirements.txt

lint:
	python -m black . --check

format:
	python -m black .

test:
	python -m pytest

bump-patch:
	bump2version patch --commit

bump-minor:
	bump2version minor --commit

bump-major:
	bump2version major --commit

bump-dev:
	bump2version patch
	bump2version --allow-dirty --new-version $(shell python setup.py --version).dev0+$(shell git rev-parse --short HEAD) commit --commit --message "For dev release only"

publish-archive:
	git archive -o latest.zip HEAD
	aws s3 cp latest.zip s3://public-artefacts-releases/artefacts-client-latest.zip
	aws s3 cp latest.zip s3://public-artefacts-releases/pep503/artefacts-client/artefacts-client-$(shell python setup.py --version).zip
	aws s3 ls s3://public-artefacts-releases/pep503/artefacts-client/artefacts-client | awk 'BEGIN {print "<!DOCTYPE html>\n<html>\n<body>"} {print "<a href=\"" $$4 "\">"$$4"</a>"} END {print "</body>\n</html"}' > client_index.html
	aws s3 cp client_index.html s3://public-artefacts-releases/pep503/artefacts-client/index.html
	aws cloudfront create-invalidation --distribution-id E3K9A05F2MA1R1 --paths "/pep503/artefacts-client/*"


publish-dev-archive:
	git archive -o dev.zip HEAD
	aws s3 cp dev.zip s3://public-artefacts-releases/artefacts-client-dev.zip
	aws s3 cp dev.zip s3://public-artefacts-releases/artefacts-client-$(shell python setup.py --version).zip

publish-docs:
	mkdocs build
	aws s3 sync site s3://docs.artefacts.com/latest
	aws s3 sync site s3://docs.artefacts.com/$(shell python setup.py --version | rev | cut -d"." -f2-  | rev)
	aws cloudfront create-invalidation --distribution-id E26DD6TYSR4ZW2 --paths "/*"

publish-dev-docs:
	mkdocs build
	aws s3 sync site s3://docs.artefacts.com/dev
	aws cloudfront create-invalidation --distribution-id E26DD6TYSR4ZW2 --paths "/dev/*"

