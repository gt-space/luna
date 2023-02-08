import "../App.css";
import { SimpleTitleBar } from "../window-components/TitleBar";
import MenuBar from "./MenuBar";
import Body from "./Body";
import Footer from "../window-components/Footer";

function Taskbar() {
  return <div class="taskbar">
    <div>
      <SimpleTitleBar/>
    </div>
    <div>
      <MenuBar/>
    </div>
    <Body/>
    <Footer/>
  </div>
}

export default Taskbar;
