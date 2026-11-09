"""ROS 2 panel: the nodes and topics of the running system (like `ros2 node info` / `ros2 topic list`)."""

from ..qt import Qt, QtWidgets


class GraphPanel(QtWidgets.QWidget):
    def __init__(self, node_info, parent=None):
        """node_info(full_name) -> {category: [(name, [types])]}, e.g. RosBridge.node_info."""
        super().__init__(parent)
        self._node_info = node_info
        self._graph = {"nodes": [], "topics": []}
        self._shown_info = None
        layout = QtWidgets.QVBoxLayout(self)

        self._filter = QtWidgets.QLineEdit()
        self._filter.setPlaceholderText("Φίλτρο (π.χ. arm, joint, camera)…")
        self._filter.setClearButtonEnabled(True)
        self._filter.textChanged.connect(self._refresh)
        layout.addWidget(self._filter)

        self._tabs = QtWidgets.QTabWidget()
        layout.addWidget(self._tabs, 1)

        split = QtWidgets.QSplitter(Qt.Horizontal)
        self._nodes = QtWidgets.QListWidget()
        self._nodes.currentTextChanged.connect(self._show_node)
        self._info = QtWidgets.QTreeWidget()
        self._info.setHeaderLabels(["Όνομα", "Τύπος"])
        self._info.setColumnWidth(0, 260)
        split.addWidget(self._nodes)
        split.addWidget(self._info)
        split.setStretchFactor(1, 2)
        self._tabs.addTab(split, "Nodes")

        self._topics = QtWidgets.QTableWidget(0, 4)
        self._topics.setHorizontalHeaderLabels(["Topic", "Τύπος", "Pub", "Sub"])
        header = self._topics.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.Stretch)
        for col in (2, 3):
            header.setSectionResizeMode(col, QtWidgets.QHeaderView.ResizeToContents)
        self._topics.verticalHeader().setVisible(False)
        self._topics.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self._topics.setAlternatingRowColors(True)
        self._tabs.addTab(self._topics, "Topics")

        self._summary = QtWidgets.QLabel("Αναμονή για το ROS 2 graph…")
        self._summary.setProperty("muted", True)
        layout.addWidget(self._summary)

    def set_graph(self, graph):
        if graph == self._graph:
            self._show_node(self._nodes.currentItem().text() if self._nodes.currentItem() else "")
            return
        self._graph = graph
        self._refresh()

    def _match(self, text):
        f = self._filter.text().strip().lower()
        return not f or f in text.lower()

    def _refresh(self):
        nodes = [n for n in self._graph["nodes"] if self._match(n)]
        current = self._nodes.currentItem().text() if self._nodes.currentItem() else None
        if [self._nodes.item(i).text() for i in range(self._nodes.count())] != nodes:
            self._nodes.blockSignals(True)
            self._nodes.clear()
            self._nodes.addItems(nodes)
            if current in nodes:
                self._nodes.setCurrentRow(nodes.index(current))
            self._nodes.blockSignals(False)

        topics = [t for t in self._graph["topics"] if self._match(t[0]) or any(self._match(x) for x in t[1])]
        self._topics.setRowCount(len(topics))
        for row, (name, types, pubs, subs) in enumerate(topics):
            for col, text in enumerate([name, ", ".join(types), str(pubs), str(subs)]):
                item = QtWidgets.QTableWidgetItem(text)
                if col >= 2:
                    item.setTextAlignment(Qt.AlignCenter)
                self._topics.setItem(row, col, item)
        self._summary.setText(f"{len(self._graph['nodes'])} nodes · {len(self._graph['topics'])} topics")

    def _show_node(self, name):
        info = self._node_info(name) if name else {}
        if (name, info) == self._shown_info:
            return
        expanded = {
            self._info.topLevelItem(i).text(0)
            for i in range(self._info.topLevelItemCount())
            if self._info.topLevelItem(i).isExpanded()
        }
        first_time = self._shown_info is None or self._shown_info[0] != name
        self._shown_info = (name, info)
        self._info.clear()
        for category, entries in info.items():
            group = QtWidgets.QTreeWidgetItem(self._info, [f"{category} ({len(entries)})"])
            for entry_name, types in entries:
                QtWidgets.QTreeWidgetItem(group, [entry_name, ", ".join(types)])
            label = category
            group.setExpanded(category in ("Δημοσιεύει", "Συνδρομές") if first_time else any(e.startswith(label) for e in expanded))
