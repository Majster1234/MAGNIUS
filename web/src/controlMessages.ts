import { EventLog, type Notice } from "./eventLog";

type Help = {title: string; description: string; remedy: string; href?: string};
export function controlSourceLabel(source:string):string {
    const names:Record<string,string>={"motion-prepare":"Przygotowanie programu","motion-preparation":"Przygotowanie programu","motion-state":"Stan wykonawcy","motion-execute":"Żądanie startu","motion-stop":"Żądanie STOP","motion-preview":"Podgląd programu","motion-connection":"Łączność z wykonawcą",simulation:"Podgląd PTP/LIN",planning:"Edycja celu",connection:"Połączenie z panelem","connection-issue":"Połączenie z panelem",freshness:"Świeżość pomiarów",axes:"Dostępność napędów","drive-enable":"ENABLE",controller:"Właściciel sterowania",planner:"Planner",jog:"Żądanie JOG","jog-stop":"Przerwanie JOG","controller-state":"Stan sterowania"};
    return names[source] ?? (source.startsWith("profile-")?"Profil i odniesienie osi":source.startsWith("operation-")?`Operacja J${source.slice(10)}`:source.startsWith("singularity-") || source.startsWith("model")?"Model 3D":source.startsWith("tcp")?"Sterowanie TCP":source.startsWith("limit")?"Ograniczenia osi":source.startsWith("motion")?"Nadzór programu":source.startsWith("simulation")?"Parametry podglądu":source);
}
const help: Record<string, Help> = {
    connection: {title:"Brak połączenia z panelem", description:"Panel nie otrzymuje aktualnego stanu sterowania. Fizyczny program może nadal trwać.", remedy:"Sprawdź połączenie LAN i stan usługi. Po powrocie łączności odczytaj stan programu przed kolejnym startem.", href:"#diagnostics"},
    freshness: {title:"Pomiary sterowania są nieaktualne", description:"Brakuje świeżych danych wymaganych do oceny położenia i gotowości osi.", remedy:"Sprawdź zasilanie napędów i diagnostykę CAN. Zaczekaj na aktualne pomiary przed przygotowaniem ruchu.", href:"#can"},
    missing: {title:"Niepełny zestaw napędów", description:"Część osi nie dostarcza pozycji. Dostępność symulacji i fizycznego wykonania jest sprawdzana oddzielnie.", remedy:"Sprawdź zasilanie i połączenia wymienionych osi. Próbę niepełnego zestawu wykonuj wyłącznie w świadomie wybranym trybie.", href:"#diagnostics"},
    enable: {title:"Napędy nie są załączone", description:"Wyłączony lub nieznany ENABLE uniemożliwia fizyczny ruch tych osi.", remedy:"Sprawdź gotowość napędów w trybie LIVE i świadomie załącz wymagane osie. Następnie przelicz i przygotuj program."},
    calibration: {title:"Odniesienie osi niepotwierdzone", description:"Kąt przegubu nie ma ważnego odniesienia do licznika wału. Ruch tej osi jest zablokowany.", remedy:"W postoju przy ENABLE=0 sprawdź fizyczną pozycję i offset w Parametrach robota, a następnie potwierdź odniesienie wskazanej osi.", href:"#config"},
    profile: {title:"Napęd wymaga weryfikacji", description:"Profil osi jest zapisany, ale przygotowanie napędu nie zostało potwierdzone.", remedy:"Sprawdź stan i opis osi w LIVE. Przygotowanie programu ponownie zweryfikuje wymagane ustawienia napędu."},
    configuration: {title:"Zmienił się kontekst planowania", description:"Poprzedni plan nie odpowiada aktualnemu modelowi, konfiguracji lub ustawieniom.", remedy:"Sprawdź zastosowaną konfigurację, skopiuj aktualną pozycję robota i ponownie oblicz oraz przygotuj PTP lub LIN.", href:"#config"},
    program: {title:"Program wymaga ponownego przygotowania", description:"Wybrany program lub jego pozycja startowa nie odpowiada bieżącemu stanowi robota.", remedy:"Odczytaj aktualne pozycje, ponownie oblicz podgląd i przygotuj program. Sprawdź nowe osie oraz start i cel przed zatwierdzeniem."},
    limits: {title:"Cel lub parametry poza limitami", description:"Co najmniej jedna pozycja, prędkość albo wartość przyspieszenia nie spełnia ograniczeń profilu.", remedy:"Popraw wskazane pole lub cel, zachowując limity mechaniczne i jednostki. Następnie przelicz podgląd."},
    unreachable: {title:"Nie można wyznaczyć drogi do celu", description:"Planner nie znalazł osiągalnego rozwiązania IK lub ciągłej drogi dla wybranego ruchu.", remedy:"Zmień pozycję lub orientację celu. Dla LIN sprawdź całą prostą TCP, a następnie ponownie oblicz podgląd."},
    singularity: {title:"Blisko osobliwości modelu", description:"W tej pozycji niewielka zmiana TCP może wymagać dużych zmian kątów przegubów. Ocena pochodzi z modelu URDF.", remedy:"Odsuń cel lub zmień orientację TCP i sprawdź podgląd. W pobliżu osobliwości nadgarstka planner ogranicza prędkość J4/J6."},
    planner: {title:"Planner jest niedostępny", description:"Panel nie otrzymał potwierdzonego wyniku obliczeń trajektorii.", remedy:"Sprawdź połączenie z ROS i zgodność modelu. Po odzyskaniu dostępności ponownie oblicz podgląd.", href:"#diagnostics"},
    executor: {title:"Brak aktualnego stanu wykonawcy", description:"Panel nie może potwierdzić gotowości ani bieżącego przebiegu fizycznego programu.", remedy:"Sprawdź połączenie i diagnostykę. Zaczekaj na aktualny stan wykonawcy przed przygotowaniem lub ponowieniem startu.", href:"#diagnostics"},
    jog: {title:"Komenda JOG odrzucona", description:"Sterowanie nie zaakceptowało żądanego ruchu osi lub celu TCP.", remedy:"Sprawdź tryb LIVE/planowanie, aktualność danych, ENABLE, odniesienie i limity wskazanej osi. Zwolnij przycisk przed kolejną próbą."},
    waiting_position: {title:"Trwa dolot do celu", description:"Wykonawca oczekuje na osiągnięcie końcowej tolerancji pozycji osi; czas wykonania może się wydłużyć.", remedy:"Obserwuj pozycje i stan napędów. Jeżeli dolot nie postępuje, użyj STOP robota i sprawdź diagnostykę przed kolejnym przygotowaniem."},
    waiting_feedback: {title:"Wykonawca oczekuje na pomiary", description:"Brakuje potwierdzonych pomiarów potrzebnych do oceny przebiegu programu.", remedy:"Sprawdź zasilanie i komunikację CAN. Brak odczytu nie potwierdza zatrzymania robota.", href:"#diagnostics"},
    waiting_budget: {title:"Wykonawca oczekuje na budżet CAN", description:"Bieżący zestaw komend czeka na dostępny budżet magistrali.", remedy:"Sprawdź obciążenie i błędy CAN. Jeżeli oczekiwanie się utrzymuje, przerwij program przyciskiem STOP i wyjaśnij przyczynę.", href:"#can"},
    partial: {title:"Tor TCP nie jest potwierdzony", description:"Program obejmuje wyłącznie wymienione osie. Niepełne pomiary nie potwierdzają osiągnięcia celu TCP.", remedy:"Sprawdź listę uczestniczących osi. Pełny ruch wymaga przywrócenia pozostałych napędów i ponownego przygotowania programu."},
    uncertain_start: {title:"Wynik żądania startu jest niepotwierdzony", description:"Brak odpowiedzi nie rozstrzyga, czy fizyczny program został uruchomiony.", remedy:"Sprawdź świeży stan programu. Nie wysyłaj nowego startu, dopóki wynik poprzedniego żądania pozostaje nieznany."},
    stop: {title:"STOP nie został potwierdzony", description:"Panel nie otrzymał potwierdzenia żądania STOP; stan ruchu pozostaje nieznany.", remedy:"Sprawdź stan robota i połączenie. W razie zagrożenia użyj dostępnego fizycznego zatrzymania stanowiska.", href:"#diagnostics"},
    fault: {title:"Awaria sterowania", description:"Właściciel sterowania zgłosił awarię. Szczegóły techniczne wskazują jej opis.", remedy:"Sprawdź stan robota oraz diagnostykę napędów i CAN. Usuń przyczynę przed ponownym przygotowaniem ruchu.", href:"#diagnostics"},
};
export function describeControlNotice(notice: Notice): Notice {
    let code=notice.code;
    if(!code && notice.severity!=="info") {
        const text=notice.message.toLowerCase();
        code=/kalibr|odniesieni|offset/.test(text)?"calibration": /osobli/.test(text)?"singularity":
            /nieosiągal|rozwiązani.*ik|ciągło.*ik/.test(text)?"unreachable": /limit|zakres|prędkość od|przyspieszenie od/.test(text)?"limits":
            /konfigur|rewiz|kontekst/.test(text)?"configuration": /program.*(nieaktual|ponownie)|pozycj.*start/.test(text)?"program":
            /niepełn|brak.*napęd/.test(text)?"missing": /planner|planer/.test(text)?"planner": /wykonawc/.test(text)?"executor":
            notice.source.startsWith("jog") || notice.source.startsWith("tcp")?"jog":undefined;
    }
    const item=code?help[code]:undefined;
    const context=notice.context ?? (notice.source.startsWith("motion")?"Robot fizyczny":notice.source.startsWith("simulation")?"Symulacja":
        notice.source.startsWith("tcp")?"Cel TCP":notice.source.startsWith("profile-")?`Oś J${notice.source.slice(8)}`:"Sterowanie");
    return {...notice,code,context,title:notice.title ?? item?.title ?? (notice.severity==="info"?notice.message:"Operacja sterowania nie powiodła się"),
        description:notice.description ?? item?.description ?? (notice.severity==="info"?notice.message:`${notice.message} Przyczyna nie została jednoznacznie rozpoznana.`),
        remedy:notice.remedy ?? item?.remedy ?? (notice.severity==="info"?undefined:"Sprawdź aktualny stan właściwej operacji i szczegóły techniczne. Przed ponowieniem ruchu wyjaśnij przyczynę odrzucenia."),
        details:notice.details ?? (notice.severity!=="info"?notice.message:undefined),href:notice.href ?? item?.href};
}
export type CurrentMessage = Notice & {kind:"condition"|"result"};
export class ControlMessages {
    readonly log=new EventLog();
    private conditions=new Map<string,Notice>();
    private results=new Map<string,Notice>();
    report(notice:Notice):boolean {
        const event=describeControlNotice(notice);
        const previous=this.results.get(event.source);
        this.results.delete(event.source);this.results.set(event.source,event);
        if(this.results.size>64)this.results.delete(this.results.keys().next().value!);
        return this.log.append(event) || previous?.progress!==event.progress;
    }
    condition(key:string,notice:Notice|null,resolution?:string):boolean {
        const before=this.conditions.get(key);
        if(!notice) {
            if(!before)return false;
            this.conditions.delete(key);
            return this.log.append({source:key,severity:"info",message:resolution ?? `${before.title} — problem ustąpił.`,context:before.context,title:resolution ?? `${before.title} — problem ustąpił.`});
        }
        const event=describeControlNotice({...notice,source:key});this.conditions.set(key,event);
        return this.log.append(event);
    }
    get active():CurrentMessage[] {
        return [...[...this.conditions.values()].map(n=>({...n,kind:"condition" as const})),
            ...[...this.results.values()].filter(n=>n.severity!=="info").map(n=>({...n,kind:"result" as const}))]
            .sort((a,b)=>Number(b.severity==="error")-Number(a.severity==="error"));
    }
    get statuses():Notice[] {return ["connection","planning","tcp-mode","simulation","motion-state","motion-preparation"].flatMap(key=>this.results.get(key) ? [this.results.get(key)!]:[]);}
}
