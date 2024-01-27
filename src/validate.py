# Copyright (c) 2024 Lee Perry

"""
This modules applies json schema validation for the complete project and node
configuration.
"""

import time

SCHEMA = {
    "type" : "object",
    "properties" : {
        "nodes" : {
            "type" : "array",
            "minItems" : 1,
            "items" : {
                "type" : "object",
                "properties": {
                    "name"      : { "type" : "string" },
                    "container" : { "type" : "string" },
                    "plugin"    : { "type" : "string" },
                    "inputs" : {
                        "type" : "array",
                        "items" : {
                            "type" : "object",
                            "properties": {
                                "name"           : { "type" : "string" },
                                "type"           : { "type" : "string" },
                                "external_topic" : { "type" : "string" },
                                "gazebo_type"    : { "type" : "string" }
                            },
                            "required" : [ "name", "type" ],
                            "additionalProperties" : False
                        }
                    },
                    "outputs" : {
                        "type" : "array",
                        "items" : {
                            "type" : "object",
                            "properties": {
                                "name"           : { "type" : "string" },
                                "type"           : { "type" : "string" },
                                "external_topic" : { "type" : "string" },
                                "gazebo_type"    : { "type" : "string" }
                            },
                            "required" : [ "name", "type" ],
                            "additionalProperties" : False
                        }
                    },
                    "environment" : {
                        "type" : "object",
                        "additionalProperties" : { "type" : "string" }
                    }
                },
                "required": [ "name", "container", "plugin" ],
                "additionalProperties" : False
            }
        },
        "connections" : {
            "type" : "array",
            "items" : {
                "type" : "object",
                "properties" : {
                    "source_node"       : { "type" : "string" },
                    "source_output"     : { "type" : "string" },
                    "destination_node"  : { "type" : "string" },
                    "destination_input" : { "type" : "string" },
                    "discard_limit"     : { "type" : "integer", "minimum" : 0 }
                },
                "required" : [ "source_node",
                               "source_output",
                               "destination_node",
                               "destination_input" ],
                "additionalProperties" : False
            }
        }
    },
    "required" : ["nodes"],
    "additionalProperties": False
}

def validate_project(project):
    """
    Validates a project and related node configuration. If the project does not
    meet the expected schema then a ValidationError exception will be raised.

    If the jsonschema package is not installed then this function will only
    print a warning, sleep for 5 seconds and exit.

    Parameters:
        project (dict) : The full project and embedded node configuration.
    """

    try:
        from jsonschema import validate
        validate(project, schema=SCHEMA)

    except ImportError:
        import logging
        logging.warn("No schema validation is possible since the jsonschema " +
                     "package is not installed! If you would like to utilise " +
                     "this feature, you must first run: \n\n" +
                     "    pip install jsonschema\n")
        time.sleep(5)
