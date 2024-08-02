import json
import yaml
import os


def update_dict(original, update):
    for key, value in update.items():
        if isinstance(value, dict):
            original[key] = update_dict(original.get(key, {}), value)
        else:
            original[key] = value
    return original


def write_file(file_path, data, format):
    with open(file_path, 'w') as f:
        if format == 'json':
            json.dump(data, f, indent=4)
        elif format == 'yaml':
            yaml.dump(data, f)


def read_file(file_path, format):
    with open(file_path, 'r') as f:
        if format == 'json':
            return json.load(f)
        elif format == 'yaml':
            return yaml.safe_load(f)


def parse_config_file(config_file):
    with open(config_file, 'r') as f:
        content = f.read().split('\n\n')
        for segment in content:
            lines = segment.split('\n')
            file_path = lines[0]
            format = lines[1]
            data = '\n'.join(lines[2:])
            update_data = json.loads(
                data) if format == 'json' else yaml.safe_load(data)
            original_data = read_file(file_path, format)
            updated_data = update_dict(original_data, update_data)
            write_file(file_path, updated_data, format)


parse_config_file('config.txt')
