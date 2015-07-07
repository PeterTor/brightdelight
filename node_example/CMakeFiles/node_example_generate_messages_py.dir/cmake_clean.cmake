FILE(REMOVE_RECURSE
  "CMakeFiles/node_example_generate_messages_py"
  "devel/lib/python2.7/dist-packages/node_example/msg/_NodeExampleData.py"
  "devel/lib/python2.7/dist-packages/node_example/msg/__init__.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/node_example_generate_messages_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
