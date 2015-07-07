FILE(REMOVE_RECURSE
  "CMakeFiles/node_example_gencfg"
  "devel/include/node_example/nodeExampleConfig.h"
  "devel/share/node_example/docs/nodeExampleConfig.dox"
  "devel/share/node_example/docs/nodeExampleConfig-usage.dox"
  "devel/lib/python2.7/dist-packages/node_example/cfg/nodeExampleConfig.py"
  "devel/share/node_example/docs/nodeExampleConfig.wikidoc"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/node_example_gencfg.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
