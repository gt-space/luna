import Footer from "../window-components/Footer";
import { GeneralTitleBar } from "../window-components/TitleBar";
import SideNavBar from "./SideNavBar";
import SystemMainView from "./SystemMainView";

function System() {
  return <div class="system">
    <div style="height: 60px">
      <GeneralTitleBar name="System"/>
    </div>
    <div class="system-body">
      <SideNavBar/>
      <div style="display: grid; grid-template-rows: 20px 1fr 25px; height: 100%">
        <div></div>
        <div class="vertical-line-2"></div>
        <div></div>
      </div>
      <SystemMainView/>
    </div>
    <Footer/>
</div>
}

export default System;
