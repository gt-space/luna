/* @refresh reload */
import { render } from "solid-js/web";
import "../style.css";
import "../App.css";
import CautionWarningWindow from "./CautionWarningWindow";

render(() => <CautionWarningWindow />, document.getElementById("root") as HTMLElement);
