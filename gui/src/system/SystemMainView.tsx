import { Component} from "solid-js";
import { Connect, Feedsystem, ConfigView, Sequences, Triggers } from "./SystemPages";
// import { Connect, Feedsystem, ConfigView, Sequences } from "./SystemPagesNew";
import { currentPage } from "./SideNavBar";

function displayPage(page: string) {
  switch(page){
    case 'side-nav-connect': return <Connect/>
    case 'side-nav-feedsystem': return <Feedsystem/>
    case 'side-nav-config': return <ConfigView/>
    case 'side-nav-sequences': return <Sequences/>
    case 'side-nav-triggers': return <Triggers/>
  }
}

const SystemMainView: Component = (props) => {
  return <div class="system-main-view">
    {displayPage(currentPage())}
</div>
}

export default SystemMainView;