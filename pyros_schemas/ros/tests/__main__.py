import pytest

# Simple way to launch all tests
if __name__ == '__main__':
    pytest.main([
        '-s',
        'test_basic_fields.py::test_field_deserialize_serialize_from_ros_inverse',
        'test_basic_fields.py::test_field_serialize_deserialize_from_py_inverse',
        'test_schema.py::test_schema_load_dump_fromros_inverse',
        'test_schema.py::test_schema_dump_load_frompy_inverse',
    ])
