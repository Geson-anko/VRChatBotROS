FILE=. 

.PHONY: tests

tests:
	isort ${FILE}
	black ${FILE}
	docformatter -r --in-place ${FILE} 

format:
	isort ${FILE}
	black ${FILE}
	docformatter -r --in-place ${FILE} 