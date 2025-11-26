# Hardware-in-the-Loop (HIL) Testing Framework
luna/sam$ maturin develop --features "python test_mode" --target x86_64-unknown-linux-gnu
OR
luna/sam$ maturin develop --target x86_64-unknown-linux-gnu

~~~~~~~~~~~~~~~~~
in ubuntu wsl
From luna
./.venv/bin/python -m pytest -v tests/hil/sam

from luna/sam
maturin develop --target x86_64-unknown-linux-gnu
luna/sam$ maturin develop --features "python test_mode" --target x86_64-unknown-linux-gnu
