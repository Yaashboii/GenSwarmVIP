import yaml
import os


_current_dir = os.path.dirname(os.path.abspath(__file__))
_config_path = os.path.join(_current_dir, '../../config/api_data/keys.yml')

with open(_config_path, 'r') as config_file:
    config = yaml.safe_load(config_file)

api_base = config.get('api_base', '')
api_keys = config.get('api_keys', {})