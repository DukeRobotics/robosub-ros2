python
# Enable pretty printers for STL types in GDB
# This script adds the GCC libstdc++ pretty printer path to GDB's Python environment,
# and registers pretty printers for STL containers (e.g., std::vector, std::map, std::unordered_map),
# making them display in a human-readable format during debugging.
import sys
sys.path.insert(0, '/usr/share/gcc/python')
from libstdcxx.v6.printers import register_libstdcxx_printers
register_libstdcxx_printers (None)
end