import { Component, createSignal} from "solid-js";

export const [currentPage, setCurrentPage] = createSignal('side-nav-connect');

function updateSelection(selection: string){
  if (selection != currentPage()) {
    document.getElementById(currentPage())!.style.backgroundColor = '#3C3F41';
    document.getElementById(selection)!.style.backgroundColor = '#333333';
    setCurrentPage(selection);
  }
}

const SideNavBar: Component = (props) => {
  return <div class="side-nav-bar">
    <div style="background-color: #333333" id="side-nav-connect" class="side-nav-button" onClick={() => updateSelection("side-nav-connect")}>
      Connect
    </div>
    <div id="side-nav-feedsystem" class="side-nav-button" onClick={() => updateSelection("side-nav-feedsystem")}>
      Setup
    </div>
    <div id="side-nav-config" class="side-nav-button" onClick={() => updateSelection("side-nav-config")}>
      Config
    </div>
    <div id="side-nav-sequences" class="side-nav-button" onClick={() => updateSelection("side-nav-sequences")}>
      Sequences
    </div>
</div>
}

export default SideNavBar;
