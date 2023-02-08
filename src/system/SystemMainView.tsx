import { Component} from "solid-js";
import { Connect, Feedsystem, Config, Sequences } from "./SystemPages";
import { currentPage } from "./SideNavBar";

function displayPage(page: string) {
  switch(page){
    case 'side-nav-connect': return <Connect/>
    case 'side-nav-feedsystem': return <Feedsystem/>
    case 'side-nav-config': return <Config/>
    case 'side-nav-sequences': return <Sequences/>
  }
}

const SystemMainView: Component = (props) => {
  return <div class="system-main-view">
    {displayPage(currentPage())}
</div>
}

export default SystemMainView;