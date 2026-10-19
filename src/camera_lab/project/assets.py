"""Local resource resolution. Package mappings do not require ROS."""
from pathlib import Path
from urllib.parse import urlparse, unquote


def resolve_resource(value, base, packages):
    if value.startswith('package://'):
        uri=urlparse(value)
        if uri.netloc not in packages:
            raise ValueError(f'Unmapped package {uri.netloc!r}; select its directory with Map package')
        path=Path(packages[uri.netloc])/unquote(uri.path).lstrip('/')
    elif value.startswith('file://'):
        uri=urlparse(value)
        if uri.netloc not in ('','localhost'):
            raise ValueError(f'Only local file resources are supported: {value}')
        path=Path(unquote(uri.path))
    elif '://' in value:
        raise ValueError(f'Unsupported resource URI: {value}')
    else:
        path=Path(value)
        if not path.is_absolute():
            path=Path(base)/path
    path=path.expanduser().resolve()
    if not path.is_file():
        raise FileNotFoundError(f'Missing resource: {path}')
    return str(path)
