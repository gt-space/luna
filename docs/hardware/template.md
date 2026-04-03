# Hardware Description Template

Hardware description documents (HDDs) are formal specifications
intended to reflect the exact hardware configuration of a particular revision of
a self-contained hardware system, typically a single board, as a reference for
software development for that platform.

HDDs are intended to be read both by human developers and AI agents. As such,
they must be:
- **Complete.** Hardware descriptions must be complete and fully self-contained
  within the HDD. External links may be included only for reference to external
  hardware.
- **Compact.** HDDs must not be excessively long or convoluted. They are a
  specification that solely contains all information relevant to software
  implementation, not a full categorization of every hardware design decision
  and component on the board.
- **Current.** HDDs must be updated to reflect changes in hardware, whether
  in Altium or ad-hoc, in a timely manner, to ensure that software is being
  implemented based on the latest and most correct hardware description.
- **Constant.** HDDs should not be modified with a new revision of hardware. A
  new HDD should be created, perhaps by duplicating and then modifying the old
  one if there are many similarities.
