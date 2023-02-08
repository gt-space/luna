/* @refresh reload */
import { render } from "solid-js/web";

import "../style.css";
import SystemPage from "./SystemPage";

render(() => <SystemPage />, document.getElementById("root") as HTMLElement);
