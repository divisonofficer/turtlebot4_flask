import { HStack, VStack, useMediaQuery } from "@chakra-ui/react";
import React, { useState } from "react";
import { H3, H4 } from "../design/text/textsystem";

import { useNavigate } from "react-router-dom";
import { MenuItem, menuList } from "../static/MenuList";

const MenuButton = ({ menu, level }: { menu: MenuItem; level: number }) => {
  const naviate = useNavigate();

  const [isHovered, setIsHovered] = useState(false);

  const isPresentPage =
    menu.destination && window.location.pathname === "/#" + menu.destination;

  console.log(window.location.pathname, menu.destination, isPresentPage);

  return (
    <>
      <HStack
        style={{
          marginLeft: `${level + 1}rem`,
          marginRight: "1rem",
          width: `${13 - level}rem`,
          height: "3rem",
          padding: "0rem 1rem",
          justifyContent: "flex-start",
          borderRadius: "1rem",
          background: isHovered || isPresentPage ? "#1C1C1C1A" : "transparent",
        }}
        onClick={() => {
          if (menu.destination) naviate(menu.destination);
        }}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
      >
        {typeof menu.icon === "string" ? (
          <img
            src={menu.icon}
            style={{
              width: "1rem",
              height: "1rem",
            }}
            alt=""
          />
        ) : (
          <menu.icon
            style={{
              width: "1rem",
              height: "1rem",
            }}
          />
        )}
        <H4
          style={{
            fontWeight: "normal",
          }}
        >
          {menu.name}
        </H4>
      </HStack>
      {menu.subMenu &&
        menu.subMenu.map((subMenu, idx) => (
          <MenuButton menu={subMenu} key={idx} level={level + 1} />
        ))}
    </>
  );
};

const Menus = () => {
  const isMobile = useMediaQuery("(max-width: 600px)")[0];

  return (
    <HStack>
      <VStack
        style={{
          width: "15rem",
          height: isMobile ? "auto" : "100vh",
          justifyContent: "flex-start",
          alignItems: "flex-start",
        }}
      >
        <H3
          style={{
            marginLeft: "1rem",
            marginTop: "3rem",
          }}
        >
          Menu
        </H3>
        {menuList.map((menu, idx) => (
          <MenuButton menu={menu} key={idx} level={0} />
        ))}
      </VStack>

      <div
        style={{
          width: "1px",
          height: isMobile ? "auto" : "100vh",
          background: "#1C1C1C1A",
        }}
      />
    </HStack>
  );
};

const MenuToggle = () => {
  const [isOpen, setIsOpen] = React.useState(false);
  const [isHovered, setIsHovered] = useState(false);
  const handleToggle = () => {
    setIsOpen(!isOpen);
  };

  return (
    <VStack>
      <HStack
        style={{
          marginLeft: "1rem",
          marginRight: "1rem",
          width: "13rem",
          height: "3rem",
          padding: "0rem 1rem",
          justifyContent: "flex-start",
          borderRadius: "1rem",
          background: isHovered ? "#1C1C1C1A" : "transparent",
        }}
        onMouseEnter={() => setIsHovered(true)}
        onMouseLeave={() => setIsHovered(false)}
        onClick={handleToggle}
      >
        <H3>{isOpen ? "메뉴 닫기" : "메뉴 열기"}</H3>
      </HStack>
      <button onClick={handleToggle}></button>
      {isOpen && <Menus />}
    </VStack>
  );
};

export { Menus, MenuToggle };
