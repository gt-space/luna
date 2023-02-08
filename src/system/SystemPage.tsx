import Footer from "../window-components/Footer";
import { GeneralTitleBar } from "../window-components/TitleBar";

function SystemPage() {
  return <div class="system">
    <div style="height: 60px">
      <GeneralTitleBar name="System"/>
    </div>
    <Footer/>
</div>
}

export default SystemPage;
